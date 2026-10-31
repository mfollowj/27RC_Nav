# SCURC fast_livo 本地补丁说明（重定位恢复）

本目录保存被修改文件的**原始备份**，以及补丁内容说明。补丁目标是让 FAST-LIVO2
在 `locate_in_prior_map` 模式下**支持大范围漂移后的重定位恢复**（LIO 发散后能被新的
重定位结果拉回来），这是上游没有的能力。

被修改的文件（位于 `/home/zx/SCURC_Nav_Sim/src/src/dependencies_and_tools/fast_livo2_relocation/src/FAST-LIVO2/`）：

| 文件 | 备份 |
| --- | --- |
| `src/LIVMapper.cpp` | `LIVMapper.cpp.orig` |
| `include/LIVMapper.h` | `LIVMapper.h.orig` |

## 改了什么

### 1. 新增参数（`readParameters`）

```yaml
allow_map_reinit: true        # 是否允许用新位姿重建体素地图
map_reinit_trans_thre: 1.0    # m
map_reinit_rot_thre: 20.0     # deg
```

### 2. `initial_pose_cbk`：稳定性判据 + 触发重建

原来只是把 `/relocalization_result` 存下来。现在：

- 输入约定是 **`map ← base_link`**（由 `reloc_bridge.py` 保证）；
- 最近 5 帧彼此 ≤ 0.25 m / 8° 才算"稳定"（甩雷达时重定位输出会乱跳，不能直接用）；
- 稳定且与"已生效位姿"差 > `map_reinit_trans_thre` / `map_reinit_rot_thre` →
  置 `need_map_reinit`，下一帧重建体素地图。

### 3. `handleLIO` 初始化分支

- 条件从 `if (!lidar_map_inited)` 改为 `if (!lidar_map_inited || need_map_reinit)`；
- **把 odom 归零**（`pos_end=0`、`rot_end=I`、`vel_end=0`），让 `T_init = map←base`
  直接就是 `map→odom`。这一步必须和重建在同一帧完成：发散时速度可达数百 m/s，
  拖一帧状态就又被推走几十米；
- 重建后加**平面自检**：先验图灌进体素地图后如果一个平面都拟合不出来，明确报 ERROR
  （否则 LIO 只剩 IMU 约束，必然再次发散，且日志看起来一切正常）；
- 成功后记录 `last_applied_pose`，并等真正 Build 成功后才置 `lidar_map_inited`。

> 注意：原来在 `if (!lidar_map_inited)` 入口处就无条件 `lidar_map_inited = true`，
> 一旦下面的分支提前 `return`（先验图为空/位姿未到），之后永远不会再初始化 —— 已改为
> 初始化成功后置位。

### 4. `run()` 的等待闸门（**最关键，修段错误**）

上游写的是：

```cpp
if (locate_in_prior_map && !initial_pose_received) { ...; return; }   // 错
rclcpp::spin_some(node);
```

问题有两个：

1. `return` 会直接退出主循环，`main()` 里栈对象 `LIVMapper` 被析构，
   释放 `prior_map` 时触发 **段错误（exit code -11）**；
2. `spin_some` 在闸门之后，等待期间回调一次都不执行 → `initial_pose_cbk` 永远不会触发，
   **死锁**。

已改为：`spin_some` 放到闸门之前，闸门内 `rate.sleep(); continue;`。

### 5. 恢复时把重力转到新世界系

重置 odom 时如果只把 `rot_end` 置为单位阵、不动 `gravity`，车体一旦有 roll/pitch
（例如雷达被翻转），新世界系里的重力方向就是错的 → 重置后会**立刻再次发散**。已改为：

```cpp
_state.gravity = _state.rot_end.transpose() * _state.gravity;   // 再清零 rot/pos/vel
```

### 6. 退出汇总与退出崩溃

- 新增计数：`n_prior_seed_`（先验图播种次数）与 `n_map_reinit_`（其中漂移后重建次数），
  在 `run()` 结束时打印：
  `===== 导航结束：先验图播种 %d 次，其中重定位重建 %d 次 =====`
- `main.cpp` 改成 **堆对象且不 delete**：原先 `run()` 返回后析构 `LIVMapper`
  会在退出阶段段错误（exit code -11），现在正常退出。

## 重新编译

`fast_livo` 现在由本仓库自己的 `Deps/build_deps.sh` 编译（产物在 `Deps/install`）：

```bash
cd /home/zx/ITEM/SCU27RC_Nav
./Deps/build_deps.sh fast_livo       # 约 65 s
```

## 回滚

```bash
cd /home/zx/ITEM/SCU27RC_Nav/Deps/fast_livo
cp ../../Nav/real/patches/LIVMapper.cpp.orig src/LIVMapper.cpp
cp ../../Nav/real/patches/LIVMapper.h.orig   include/LIVMapper.h
cd /home/zx/ITEM/SCU27RC_Nav && ./Deps/build_deps.sh fast_livo
```

（`.orig` 备份在 `Nav/real/patches/`；它们是打补丁前从
`/home/zx/SCURC_Nav_Sim/.../FAST-LIVO2` 拷出来的原始文件。）

## 配套组件

- `ros2_ws/src/relocalization`：从
  `/home/zx/getting_relocal/FAST_LIVO2_ROS2_relocation_ultra` 拷入的 TEASER++/GICP
  重定位节点（只保留 `teaser_gicp_node` 和 `transform_publisher`）；
- `real/scripts/reloc_bridge.py`：把 `map←livox` 换成 `map←base`，做稳定性过滤，
  发 `/relocalization_result` 和 `map→odom`；
- `real/launch/real_reloc.launch.py` + `real_reloc.sh`：整条链路的入口。

## 已知前提

1. 先验图必须是**同一次、走遍全场**建出来的 `prior_map.pcd`，且
   `preprocess.filter_size_map` 要明显小于 `lio.voxel_size`（本工程 0.05 vs 0.2）；
2. `map←base` 的坐标系约定：`LIO 的 odom 原点在重定位瞬间被重置为机器人位姿`，
   所以 `map→odom` 恒等于最近一次被接受的 `map←base`；
3. 恢复瞬间 `map←base` 会跳变（从错误值跳到正确值），Nav2 侧需要 abort 当前目标/暂停
   controller 1~2 s。
