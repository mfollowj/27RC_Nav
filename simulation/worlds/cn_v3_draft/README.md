# 国内 V3 征求意见稿场地：Classic 与 Fortress

依据队内用户于 2026-10-25 提供的《第二十六届全国大学生机器人大赛女娲补天竞技赛规则 V3—征求意见稿》制作。原资料是草案，文件修订表还有日期矛盾；本目录不表示正式 V3 规则已发布。

这是仓库 ROS 2 Humble / Gazebo Classic 11 链路的可选场地；同目录另有 Fortress 独立场地入口。默认 `legacy` 入口、旧地图和标定保持原样。完整使用说明见 [地图与场地更新](../../../docs/maps.md)。

```bash
ros2 launch rc_field_nav sim.launch.py field:=cn_v3_draft gui:=true
```

当前 `rc27_nav` Docker 实际安装的是 Ignition Gazebo 6.18.0（Fortress），没有 Classic 11。仅展示场地及道具可在该环境运行：

```bash
cd simulation/worlds/cn_v3_draft   # 从仓库根目录进入场地目录
IGN_PARTITION=scu27rc_v3_field_demo ./start_fortress_field.sh
```

脚本直接使用 `source/initial_objects.world` 与便携模型，自设资源路径；不启动机器人、ROS、SLAM 或导航。需要无 GUI 时追加 `-s`，可用 `--iterations 10` 运行有限步。与现场同时使用时，为展示场地单独指定 `IGN_PARTITION`。

场地内净 11×11 m，红方 x<0、北方 +y；地面顶 z=.025 m，L1/L2 顶 .625/.925 m。L1 栏厚/高各 50 mm，L1→L2 直接高差 300 mm，无中间踏步，下层 6 踏面保留。场地有 97 visual / 79 collision；配套初态道具为红蓝大块各 20、塔顶 12、球 1。

`worlds/robocon_cn_v3_draft.world` 含场地、53 道具与沿用旧场景的场外辅助地面；`worlds/robocon_cn_v3_draft_field_only.world` 不含比赛道具。两者均不内置机器人，现有 `sim.launch.py` 仍生成仓库的 omni4。辅助地面用于承托出场物体，不是赛事场馆尺寸。

维护源位于 `source/`：

- `models/`：从队内 2026-10-30 V3 草案可分发场地包导入的便携模型模板，保留网格与道具质量/惯性。
- `initial_objects.world`：同一分发包的模型初态模板；保留 Fortress 原格式用于追溯，仅模型 include 被导入 Classic。
- `gui/field.config`：同一分发包的 Fortress 场地总览视角。
- `classic_world_base.world`：从本仓库旧 `robocon_track.world` 提取的 Classic ODE、ROS state、光照及场外地面设置，不含旧场地 include。

`generate_classic.py` 只使用 Python 标准库。它从上述模板生成 `models/` 和 `worlds/`，将模型声明为 SDF 1.6，并使用 Classic 世界设置，不加载 Fortress 的 Ignition 系统插件。资源 URI 均为包内 `model://`，launch 自动设置 `GAZEBO_MODEL_PATH`；不依赖原作者目录或在线模型库。

```bash
python3 simulation/worlds/cn_v3_draft/generate_classic.py
```

更新几何时先导入对应规则版本的新模板，再运行生成器；不要单改生成的 `models/`、`worlds/`。CMake 将本目录安装到 `share/rc_field_nav/worlds/cn_v3_draft`，修改源后重新构建安装。

已在 `rc27_nav` 的独立 Fortress server 实际加载 54 场地/道具模型并执行 10 个物理步，退出码 0。Classic 资源已完成 CMake 构建安装和静态检查，尚未在 Classic server 实测。当前移植保留了原型摩擦与球托碰撞近似；两种引擎接触不等价。这些结果不代表导航地图、动态避障、坡道或跨层能力已验证。
