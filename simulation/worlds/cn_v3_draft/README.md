# 国内 V3 征求意见稿场地与 BR／TR

依据队内用户于 2026-10-25 提供的《第二十六届全国大学生机器人大赛女娲补天竞技赛规则 V3—征求意见稿》制作。原资料是草案，文件修订表还有日期矛盾；本目录不表示正式 V3 规则已发布。

本目录提供 Fortress 双车、Fortress 仅场地，以及原 ROS 2 Humble / Gazebo Classic 11 的 omni4 场地资源。完整说明见 [地图与场地更新](../../../docs/maps.md)。

## Fortress：V3 场地与 BR／TR

从仓库根目录使用现有 `rc27_nav` 容器：

```bash
./sim_robots.sh --docker rc27_nav
# 无界面有限步运行
./sim_robots.sh --docker rc27_nav -s --iterations 1000
```

该容器安装 Ignition Gazebo 6.18.0（Fortress）。入口只复制本目录及 `simulation/robots` 到私有临时目录，前台运行后清理，不改变已有挂载；本机已有 Fortress 时运行 `./sim_robots.sh` 即可，无需编译 ROS 源码。

`start_fortress_robots.sh` 加载 `worlds/robocon_cn_v3_draft_tr_br.world`：54 个场地／道具模型加 BR／TR，共 56 个模型。两车为队内视频启发重建的 `video_v2` 仿真原型，保留 BR 麦轮滚子、TR 普通轮和各自机构；源、控制话题与生成方法见 [机器人模型说明](../../robots/README.md)。双车初态由 `generate_dual_robots.py` 生成，准确坐标见地图文档。

入口使用独立 `IGN_PARTITION`，保留 `/br_v2`、`/tr_v2` 原生控制话题，默认不启 GPU Sensors，也不启动 ROS、LIO 或 Nav2。已有 56 模型原生 SceneInfo 及 1000 物理步退出码 0 的实际结果，详见 [验证记录](../../../docs/status.md)。

## Fortress：仅场地与道具

```bash
cd simulation/worlds/cn_v3_draft   # 从仓库根目录进入场地目录
IGN_PARTITION=scu27rc_v3_field_demo ./start_fortress_field.sh
```

脚本直接使用 `source/initial_objects.world` 与便携模型，自设资源路径；不启动机器人、ROS、SLAM 或导航。需要无 GUI 时追加 `-s`，可用 `--iterations 10` 运行有限步。与现场同时使用时，为展示场地单独指定 `IGN_PARTITION`。

## Classic：原 omni4 导航场地

Classic 11 与 ROS 插件环境下，安装本包并运行：

```bash
source env.sh
ros2 launch rc_field_nav sim.launch.py field:=cn_v3_draft gui:=true
```

该入口仍生成 omni4；默认 `legacy`、旧地图和标定保持原样。切到 V3 导航前按地图文档重新制作匹配的 PCD 与二维图。Fortress 双车 world 使用 Ignition 插件，不能直接作为此 Classic 入口的 world。

场地内净 11×11 m，红方 x<0、北方 +y；地面顶 z=.025 m，L1/L2 顶 .625/.925 m。L1 栏厚/高各 50 mm，L1→L2 直接高差 300 mm，无中间踏步，下层 6 踏面保留。场地有 97 visual / 79 collision；配套初态道具为红蓝大块各 20、塔顶 12、球 1。

`worlds/robocon_cn_v3_draft.world` 含场地、53 道具与沿用旧场景的场外辅助地面；`worlds/robocon_cn_v3_draft_field_only.world` 不含比赛道具。这两个 Classic world 均不内置机器人，由 `sim.launch.py` 生成 omni4。辅助地面用于承托出场物体，不是赛事场馆尺寸。

## 生成与维护

维护源位于 `source/`：

- `models/`：从队内 2026-10-30 V3 草案可分发场地包导入的便携模型模板，保留网格与道具质量/惯性。
- `initial_objects.world`：同一分发包的模型初态模板；保留 Fortress 原格式用于追溯，仅模型 include 被导入 Classic。
- `gui/field.config`：同一分发包的 Fortress 场地总览视角。
- `classic_world_base.world`：从本仓库旧 `robocon_track.world` 提取的 Classic ODE、ROS state、光照及场外地面设置，不含旧场地 include。

`generate_classic.py` 只使用 Python 标准库。它从上述模板生成 `models/` 和 `worlds/`，将模型声明为 SDF 1.6，并使用 Classic 世界设置，不加载 Fortress 的 Ignition 系统插件。资源 URI 均为包内 `model://`，launch 自动设置 `GAZEBO_MODEL_PATH`；不依赖原作者目录或在线模型库。

```bash
python3 simulation/worlds/cn_v3_draft/generate_classic.py
# Fortress 双车场景组合与出生点
python3 simulation/worlds/cn_v3_draft/generate_dual_robots.py
```

更新几何时先导入对应规则版本的新模板，再运行对应生成器；不要单改生成的 `models/`、`worlds/`。修改 BR／TR 几何与机构时按机器人模型说明更新生成源并重新生成模型。CMake 将场地目录安装到 `share/rc_field_nav/worlds/cn_v3_draft`；Classic 使用安装资源，修改后重新构建安装，根 Fortress 入口使用源码目录。

先前仅场地入口已在独立 Fortress server 加载 54 模型并执行 10 步；本轮双车入口另完成 56 模型与 1000 步测试。Classic 资源完成 CMake 构建安装和静态检查，尚未在 Classic server 实测。摩擦与球托碰撞沿用原型近似，长时间接触、坡道、跨层、机构任务和导航链需各自验证。
