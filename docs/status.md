# 项目当前情况

更新时间：2026-10-31。运行步骤见 [上手与部署](getting_started.md)，修改路径见 [源码与配置索引](source_index.md)。

## 当前代码与验证范围

| 模块 | 当前代码 | 验证与下一步 |
| --- | --- | --- |
| 构建与环境 | 根 `build.sh` 统一构建；`env.sh` 加载 Humble、`third_party/install`、`install` | 目标系统仍需 ROS／Gazebo／Nav2、系统库与 Livox-SDK2；换机重新构建 |
| 仿真导航 | FAST-LIVO2、TEASER++／VGICP、bridge、Nav2，保留旧定位实现 | 下方保存 2026-10-30 原场地闭环记录；本轮改动的运行结果另行记录 |
| 国内 V3 场地 | 显式 `field:=cn_v3_draft`；默认保留 legacy 场地 | Docker Fortress 已实际加载 54 个模型并执行 10 步；Classic 资源安装与 launch 选择通过，整车链待测，详见 [地图文档](maps.md) |
| 真机导航 | `real_map.sh` 建图，`real_reloc.sh` 仅定位，`real_nav.sh` 带 Nav2 | 目标机重做／确认全场先验图，再验证定位与底盘执行 |
| 小机器人位置串口 | 根 `start_robot.sh` → `robot/start_robot.sh`；本仓库驱动、LIO、mcu_bridge | 使用原始 IMU 和 `odom→base_link`；先 dry-run，再与电控验证 |
| 决策层 | `decision/` 现有 5 份 Markdown 设计 | 比赛节点、任务 launch 与机构执行接口待实现 |

### 本轮修正与仍需处理的问题

- 实际目录已调整为根 `src/`、`robot/`、`config/`、`simulation/`、`third_party/`、`docs/`、`decision/`；项目在根目录构建到 `build/install/log`，旧新文件名见 [迁移表](naming.md)。包名、节点接口与根脚本名称保留。
- 文档与源码按现有 4 个项目包分类，构建与索引包含 `mcu_bridge`；小机器人根入口与真实脚本路径统一。
- 小机器人入口改为仓库内的雷达配置和 `third_party/install`，当前共用 YAML 的 `extrinsic_R` 为单位阵，订阅 `/livox/imu`。下方旧记录中的 `rot_imu`、`camera_init`、`Deps/livo_install` 属于当时的部署情况。
- Nav2 的 bt_navigator、controller_server、velocity_smoother 的 odom 话题已对齐实际 LIO 输出 `/aft_mapped_to_init`；仿真轮速里程计 `/odom_gz` 用于对照。
- **速度反馈仍待补充／验证。** 当前 `third_party/fast_livo/src/LIVMapper.cpp` 的 `publish_odometry` 填充 pose，twist 仍为默认零；Nav2 的速度反馈需要真实 twist 支持，并完成仿真与真机闭环验证。话题路径修正只解决数据源名称。
- Nav2 输出速度和 MCU 位置闭环是两条执行方式。默认 `real_nav.sh` 没有启动真机串口速度消费者，按电控接口接入后再进行实车导航。
- 切到 V3 场地要重建同一坐标系下的 PCD 与二维图；当前保留的旧先验图不自动升级。

### 本轮实际验证（2026-10-31，含目录迁移前记录）

以下为本轮改动后的执行结果，与后文历史记录分开维护：

| 执行内容 | 结果与范围 |
| --- | --- |
| 新目录完整真机构建 | 旧产物移出后，在全新 ASCII 构建目录实际运行 `JOBS=1 ./build.sh robot`，整体退出码 0；依赖安装到 `third_party/install`，两个项目包安装到根 `install`。4 个二进制动态库解析、Livox 消息支持、MCU 导入和现有帧测试通过 |
| 真机统一构建 | `JOBS=1 ./build.sh robot` 实际整体退出码 0：small_gicp、TEASER++、Livox 驱动、vikit 三个包、FAST-LIVO2，以及 mcu_bridge／relocalization 编译安装通过；接口生成使用 ASCII 构建路径 |
| ROS 包与资源安装 | `mcu_bridge` 普通 colcon 安装通过；`rc_field_nav` 独立构建安装及 V3 资源解析通过 |
| Nav2 配置 | 在独立 DDS 域实际配置 controller_server 并读取参数，`odom_topic=/aft_mapped_to_init`；节点保持 inactive |
| 串口参数优先级 | 实际启动 dry-run 节点，验证自定义 YAML 的串口、波特率和坐标系生效，显式 launch 参数可单独覆盖；未打开串口 |
| 位置帧与脚本 | 现有 `test_frame.py` 通过；修改的 shell/Python 语法、环境重复加载、入口参数和文档本地链接检查通过 |
| Docker V3 场地 | `rc27_nav` 的 Fortress 6.18.0 在独立 partition 加载场地及 53 道具，10 步后模型数仍为 54；独立场地脚本退出码 0 |
| 目录迁移后的仿真资源 | 根 `src/rc_field_nav` 在新临时前缀真实安装通过；资源 URI、场景选择、建图与导航 LaunchDescription 构造通过。沿用同一资产的 Fortress 测试结果，本次未重做 Docker 运行 |

新目录还实际通过真机 launch 和根入口的参数展开、环境重复加载，以及路径含空格的离线存图；存图检查使用临时输入与输出，不覆盖仓库地图。

当前机器没有 Gazebo Classic 11；Fortress 场地短测不覆盖 Classic 整车、雷达、LIO、Nav2 的联动。真机雷达、串口发送与运动验证留待对应硬件上执行。

## 历史运行与联调记录（2026-10-30～31）

以下保留原有测量、部署现象和后续计划，便于追溯。数字、机器路径、服务安装与“已完成”描述均对应记录当时的条件；当前入口与参数按上方状态及运行文档使用。

### 一、目标拆解

| 里程碑 | 内容 | 状态 |
| --- | --- | --- |
| M1 仿真闭环 | Gazebo 场地 + BR（真实尺寸/质量/雷达参数）+ LIO + Nav2 | **可用**（2026-10-30 全流程实测：建图 → 存图 → 重定位 → Nav2 到位） |
| M2 真机闭环 | Mid-360 + FAST-LIVO2 + 6-DOF 重定位 + Nav2 | 可用（**P2/P3 改动后尚未上机复验**） |
| M3 感知统一 | 仿真与真机**同一套 LIO（FAST-LIVO2 无视觉）、同一套话题、同一份参数** | **完成**（代码 + 仿真实测；真机待复验） |
| M4 决策层 | Decide：任务状态机 / 行为树 + Nav2 action | **框架 + 可行性完成**（2026-10-31，文档；代码未开始） |
| M5 全场稳定 | 建图、重定位、漂移恢复在整场范围内稳定 | 进行中（卡在真机先验图） |

### 二、各模块现状

#### 环境自包含（P2，2026-10-30）

- ✅ `./Deps/build_deps.sh` 全量编译通过：small_gicp / TEASER++ / vikit_* /
  livox_ros_driver2 / fast_livo / fast_lio，全部装进 `Deps/install`。
- ✅ `env.sh` 删掉了 `/home/zx/SCURC_Nav_Sim` 与 `/home/rc/*` 的兜底；
  `real_reloc.sh` / `real_reloc.launch.py` 删掉了 `/home/zx/getting_relocal` 的兜底。
- ✅ `env.sh` 会**主动清洗终端里继承来的旧工作区前缀**（旧工作区在别的终端 source 过时，
  同名包会被它们抢先匹配），并统一补 `Deps/install/lib` 到 `LD_LIBRARY_PATH`。
- ✅ `Nav/ros2_ws` 已在干净环境里重建（`colcon build --symlink-install`），
  `teaser_gicp_node` 链接的是 `Deps/install/lib/libteaser_*.so`。
- 验证：`source env.sh` 后 `fast_livo / fast_lio / livox_ros_driver2` 都解析到
  `Deps/install`，`nav2_bringup` 解析到 `/opt/ros/humble`，`ldd` 无 `not found`。

#### Nav/config（M3 的落点）

- ✅ `Nav/config/livo_mid360.yaml`：**real 与 sim 唯一**的 FAST-LIVO2 参数文件。
  （原来的 `Nav/real/config/livo_mid360_map.yaml`、`livo_mid360_locate.yaml` 已删除，
  `camera_pinhole.yaml` 也移到 `Nav/config/`。）
- 平台差异只有两处，写在 launch 里：
  外参（真机出厂值 vs 仿真 `[0,0,0.06]`）与 `use_sim_time`。
- **小机器人默认启动**（2026-10-31）：`Nav/real/start_robot.sh`（机器人上 `~/SCU27RC_Nav/start_robot.sh`）
  取代了 SCURM 的 `position_only.launch.py`（后者已在机器人上改名禁用）。
  链路：驱动(15Hz) → `cmd_chassis/rot_imu` → 抽稀 → fast_livo → mcu_bridge。
  机器人上 **用本仓库自己编的 fast_livo**（`Deps/livo_install`，与共用 YAML 配套，
  帧名 `odom->base_link`）；环境里那份 New_Nav 的 fast_livo 用同一份 YAML 会发散
  （实测位姿跑到上万米），已确认是版本差异，不是配置问题。
- **雷达/抽稀/LIO 统一 15 Hz**（2026-10-31）：驱动 `publish_freq=15`、
  `livox_preprocess.py --rate 15`、`livo_mid360.yaml` 的 `scan_rate: 15`、
  仿真 Gazebo 雷达 `update_rate=15`、`pointcloud_to_laserscan` 的 `scan_time=0.0667`。
  Mid-360 的点率是硬件固定的 20 万点/秒，15 Hz 只是把点流切成每帧约 1.3 万点；
  机器人上 SCURM 那套 fast_lio 的 `scan_rate` 也已同步改成 15（改前有备份）。

#### Nav/real（真机）

- ✅ 建图：`real_map.sh`，FAST-LIVO2 无视觉，轨迹记录支持**原地旋转**。
- ✅ 存图：`real_save_map.sh --imu-height 0.30`，离线抽稀 + `check_prior_map.py` 体检。
- ✅ 6-DOF 重定位：TEASER++（FPFH + GNC-TLS）→ VGICP，`publish_after_converged` 后每帧连续输出。
- ✅ 恢复链路：bridge 采纳 → `map→odom` 更新 → LIO `need_map_reinit` → odom 归零 + 重播种。
- ✅ 发散保护：`lio_norm > 50 m` 或残差 > 5 m/30° 时走 `LOST_RECOVER`。
- ✅ 退出汇总：重定位次数 / 首次 / 修正 / 最大残差；LIO 播种次数 / 重建次数。
- ⚠️ 先验图仍是**单视点**（21k 点、轨迹 7 点、`imu_height` 还是错的 0.003），
  走出覆盖区就退化 —— 这是现在最该修的一件事。
- ⚠️ **P2/P3 改了 env.sh、共用 yaml、launch 默认路径，真机还没复验**：
  上车第一件事是 `./real_map.sh` 能正常出图 + `./real_nav.sh nav2:=false` 能看到 FIRST。

#### Nav/sim（仿真）

- ✅ 场地 world、BR 模型、Mid-360 仿真插件（CustomMsg 与真机一致）。
- ✅ **LIO 已切到 fast_livo**（`lio_map.launch.py`），与真机同一份参数、同一个
  `livox_preprocess.py`、同一条 `/livox/lidar -> /livox/lidar_filtered` 话题链。
- ✅ **重定位已换成真机链路**（`nav_prior_map.launch.py`）：
  `teaser_gicp_node -> reloc_bridge -> /relocalization_result + map->odom`，
  LIO 走 `locate_in_prior_map`，收到稳定位姿就重建体素图。
  旧的 4-DOF `prior_map_localizer` 保留为 `reloc_impl:=voxel`（仅 A/B 对比）。
- ✅ 2026-10-30 实测：建图 → 存图 → 重定位 → Nav2 目标 **SUCCEEDED**。
- ⚠️ 仿真图的自由区还小（本次验证只覆盖约 4 m²），TEASER 的 FPFH 对应点只有
  20~70 个，所以仿真 launch 的默认阈值调成 `teaser_inlier=10 / map_voxel=0.15`
  （真机仍是 30 / 0.3）。图做大之后可以再调回去。
- ⚠️ 仿真端到端要在无 GPU 的机器上跑，需要 Mesa 软件渲染（见"五、验证环境"）。

#### Deps

- ✅ 源码随仓库打包，`build_deps.sh` 全量通过（耗时见 `Deps/README.md`）。
- ⚠️ TEASER++ 首次 configure 需要联网拉 `pmc/tinyply/spectra`（离线办法见 Deps/README）。
- ⚠️ Livox-SDK2 仍是系统级依赖（`/usr/local`），未打包。

#### Decide

- 🟡 **框架 + 可行性分析完成（2026-10-31，仅文档，代码未开始）**：
  [索引与结论](../decision/README.md) ｜ [框架设计](../decision/01_framework.md) ｜
  [接口契约](../decision/02_interfaces.md) ｜ [策略落地映射](../decision/03_strategy_mapping.md) ｜
  [可行性分析](../decision/04_feasibility.md)。
- ✅ 接口里最关键的 `/reloc_status` 已实测可用，格式：
  `mode=BOOTSTRAP icp=0 applied=0 first=0 recover=0 spread=0.00 m / 0.0 deg diff=... res_max=... lio_norm=...`
  （`mode` ∈ BOOTSTRAP / SEARCHING / APPLIED / TRACKING；**没有 LOST**，
  且首次定位前会发 identity `map→odom`，决策层不能"有 TF 就当已定位"）。
- 结论速览：安全壳 + 任务调度现在可做；真机全自动被三件事卡住 ——
  **先验图自由区仅约 0.39 m²**、**无场地语义/计分输入**、**无取石/建塔/翻顶执行接口**。
- 详细数字、风险与里程碑见 [04-可行性分析](../decision/04_feasibility.md)。

### 三、2026-10-30 的验证记录（仿真）

#### 1) 建图（fast_livo，与真机同构）

```
./sim_map.sh gui:=false rviz:=false       # 用 logs/p3_sim_drive.py 自动走场地
./sim_save_map.sh field
```

- 抽稀节点：`/livox/lidar -> /livox/lidar_filtered`，与真机同一脚本同一参数。
- LIO 累积 422 帧 / 579486 点 → 去重 47144 点；存图后 `field_map.pcd` 52999 点。
- 2D 图：147 x 146 格、0.05 m、7.4 x 7.3 m，`map_sanity_check` → **VERDICT: OK**。

#### 2) 导航（TEASER++/VGICP + reloc_bridge + fast_livo + Nav2）

```
./sim_nav.sh                              # 自动用最近一次存图（field_*）
```

- TEASER：内点 10~12 / 对应点 48~71 → 收敛；
- `reloc_bridge`：`★ 接受重定位 [FIRST]：map<-base x=-0.003 y=-0.002 z=-0.260`；
- FAST-LIVO2：`Prior map loaded: 52999 -> 9056`、`Prior map seeded OK: 611 voxels, 495 with plane`；
- `bt_navigator`：`active [3]`；`map->base_link` TF 正常；
- 目标 `(-0.09, 1.93)`：`Goal finished with status: SUCCEEDED`，
  轮速里程计从 `(0.00, 0.00)` 走到 `(-0.19, 1.95)` —— 车真的动了；
- GICP 追踪：`Converged: 1` 1667 帧 / `Converged: 0` 140 帧（后者集中在关仿真那几秒）。

### 四、已知问题清单（按优先级）

| # | 问题 | 影响 | 计划 |
| --- | --- | --- | --- |
| 1 | 真机先验图单视点、`imu_height` 写错 | 走远即漂，TEASER 内点低 | 雷达固定、慢速走遍场地重做图，`--imu-height 0.30` |
| 2 | P2/P3 改动未上机复验 | 真机可能起不来（路径/参数/库） | 上车先跑 `./real_map.sh` + `./real_nav.sh nav2:=false` |
| 3 | 首次定位无质量门控 | 可能用中偏差解播种（fitness 0.045） | 把 fitness/内点写进协方差，bridge 设阈值 |
| 4 | 发散恢复未真机验证 | 翻转雷达后能否 1 秒回位 | 下次真机翻转测试 + 看 LOST 日志 |
| 5 | 仿真地图自由区偏小 | TEASER 对应点少，要压低阈值 | 把 `logs/p3_sim_drive.py` 的路线扩到全场 |
| 6 | `--imu-height` 易写成 `0.003` | 2D 图与体检错 | 存图脚本已提示；文档强调 `0.30` |
| 7 | 旧路径软链 `RC_ground_sim` | 容易误解"改名没成功" | 全链验证后删除 |
| 8 | `fast_lio` 仍随仓库编译 | 已不被仿真使用 | 保留做对照；确认无回退需求后可移出 |

### 五、验证环境（无 GPU 的机器上跑仿真）

本次验证是在**没有 GPU 设备节点**的沙箱里做的，Gazebo 必须强制走 Mesa 软件渲染，
并且关掉在线模型库，否则会卡在 `Getting models from http://models.gazebosim.org/`：

```bash
export __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_DATABASE_URI=""
```

这些**只影响无 GPU 环境**，正常桌面（有 NVIDIA/AMD 驱动）不要设。

### 六、下一步

1. **真机复验 P2/P3**（最高优先级：改了环境脚本和 LIO 参数文件默认路径）。
2. 真机重做全场先验图（`real_save_map.sh --imu-height 0.30`），体检必须 PASS。
3. 首次定位加质量门控（fitness / 内点 → 协方差 → bridge 阈值）。
4. 真机验证雷达翻转后的 `LOST_RECOVER`。
5. 仿真地图扩到全场，把 `teaser_inlier / map_voxel` 调回接近真机的值。
6. 开始 `Decide/`：先做"收到 `/reloc_status` 的 `mode=TRACKING` 才允许发 Nav2 目标"的安全壳
   （框架与验收见 [Decide/04-可行性分析](../decision/04_feasibility.md) 的 S1）。
