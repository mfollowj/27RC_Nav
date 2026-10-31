# 地图与场地更新

仓库原有场地继续叫 `legacy`，新增 `cn_v3_draft` 可选场景，依据 2026-10-25 用户提供的国内 V3 征求意见稿。此次接入更新仿真场地资源和场景选择；现有 PCD、二维地图、LIO/Nav2 参数、外参与控制器参数没有改写。

当前 Docker `rc27_nav` 中实际是 **Ignition Gazebo 6.18.0（Fortress）**；仓库现有机器人、雷达及 ROS 启动链使用 **Gazebo Classic 11** 插件。这两套运行环境需要区分：Fortress 已完成独立场地加载与物理步验证；Classic 版本完成了资源适配、构建安装及静态验证。

## 在现有 Docker 查看独立 V3 场地

`simulation/worlds/cn_v3_draft/start_fortress_field.sh` 直接加载便携 Fortress 场地与 53 道具，不创建机器人，也不启动 LIO/Nav2。将该目录放入装有 Fortress 的容器后，在目录内运行：

```bash
IGN_PARTITION=scu27rc_v3_field_demo ./start_fortress_field.sh
```

只需无 GUI 加载时追加 `-s`；再加 `--iterations 10` 可运行 10 步后退出。脚本从自身目录定位 `source/models`、world 和总览相机；不依赖固定挂载路径。独立展示与现场仿真使用不同的 `IGN_PARTITION`。

该入口保留 Fortress 系统插件；后文 `field:=cn_v3_draft` 则属于仓库原有 Classic 整机 launch，仍需 Classic 及其 ROS 插件环境。

## 场地、点云、导航图分别负责什么

- `simulation/worlds/` 的 world / SDF / 网格决定仿真里实际存在的场地、碰撞与道具。
- `prior_map.pcd` 是采集后供重定位使用的三维先验；文件名不能说明对应规则版本。
- `*_costmap.yaml` 和配套 PGM 是 Nav2 使用的二维占据图；meta、轨迹与高度场记录同次生成信息。
- RViz 的静态边线或参考模型用于比较几何，不等于 SLAM 建图或定位修正。

更换 world 不会更新已经采集的 PCD / PGM。用于导航的三维先验与二维图应来自同一场景、同一次采集和同一坐标系。

## 为什么旧场地不直接算 V3

原 `robocon_track.world` 引用 `robocon_ground`，其模型注释与几何使用红方 `y>0`、北柱约在 `(4.15,0)`，且包含用于旧导航调试的 0.6 m 隐形边墙。新增 V3 草案采用红方 `x<0`、北方 `+y`、北柱 `(0,4.25)`，没有这些额外导航墙。旧入口和已有保护原样保留。

V3 草案的内场为 11×11 m，地面顶世界 z=.025 m；L1/L2 顶为 .625/.925 m，对应相对地面 .600/.900 m。L1 栏厚/高各 50 mm，L1→L2 为 300 mm 直接台差；中间踏步取消，下层共 6 踏面仍存在。红蓝大块各 20 个（350 mm、.600 kg），塔顶 12 个（200 mm、.220 kg），球 1 个（直径 210 mm、.270 kg）。

规则源名称明确带“征求意见稿”。PDF 内旧楼梯/足球表述与新修订条款存在冲突，模型按新专门条款及末页尺寸图处理；不将它写作正式 V3 比赛场地。

## 选择场景与出生点

按仓库现有构建流程重新安装 `rc_field_nav` 后：

```bash
source env.sh
ros2 launch rc_field_nav sim.launch.py field:=cn_v3_draft gui:=true
```

`sim.launch.py`、`sim_mapping.launch.py` 和 `sim_navigation.launch.py` 均接受 `field:=legacy|cn_v3_draft`。默认仍为 `legacy`，保持原场景和原出生点 `(-4.3,1.5,.05,0)`。显式 `world:=/绝对路径/场景.world` 优先于 field 选择。

V3 默认出生点为仓库当前 omni4 的红方启动区中心 `x=-5.15,y=-5.15,z=.05,yaw=0`。560 mm 方形车体在 700 mm 启动区内每边留 70 mm；轮底在机器人基坐标 z=0，初始离 V3 地面 25 mm。它是本仓库 omni4 的初态，不沿用其他 TR/BR 机器人的高度。`x/y/z/yaw` 可显式覆盖，建图与导航应使用相同初态。

V3 场地和 5 类模型从安装后的 `share/rc_field_nav/worlds/cn_v3_draft` 加载，无需作者机器路径。场地有 97 visual / 79 collision；默认 world 有场地 1、正式道具 53，另加沿用旧场景的辅助地面。机器人仍由原 launch 生成。只看场地可将 world 指向该目录下 `worlds/robocon_cn_v3_draft_field_only.world`。

## 在 V3 场景重新建图与存图

下列是使用现有工作流的操作步骤；本次修改未自动采集或生成 V3 先验。

```bash
./sim_map.sh field:=cn_v3_draft
```

另开终端用 `./sim_teleop.sh` 低速采集地面区域，再保留新名字存图：

```bash
./sim_save_map.sh cn_v3_draft
```

该命令使用现有保存链路输出 `cn_v3_draft_map.pcd`、轨迹及 `cn_v3_draft_costmap.*`。检查记录范围、低矮栏杆、可移动道具、禁行边界与出生点坐标后，再使用这一对先验导航；新场地没有旧模型的高隐形墙，原高度切片参数对低矮栏杆的覆盖必须实测，不能把“能存图”当作边界已可靠识别。

```bash
source env.sh
MAP_DIR="$(ros2 pkg prefix rc_field_nav)/share/rc_field_nav/maps"
./sim_nav.sh field:=cn_v3_draft \
  prior_map:="$MAP_DIR/cn_v3_draft_map.pcd" \
  map:="$MAP_DIR/cn_v3_draft_costmap.yaml"
```

`field` 不会自动重命名或替换地图参数。只传 `field:=cn_v3_draft` 而沿用旧默认 `prior_map.pcd/prior_costmap.yaml`，会把不同场景的地图混在一起。

队内此前另有从 V3 已知几何生成的 460×460、.025 m/格 PGM，限制为红方平地区域，坡道、台阶、L1、L2 和蓝方区域关闭，且不含动态道具。这类几何栅格使用场地坐标，不能替代本仓库在 LIO / 重定位坐标中采集的一套先验。因此本次没有把它作为当前导航默认图导入。

## 源文件与后续维护

[cn_v3_draft/README.md](../simulation/worlds/cn_v3_draft/README.md) 列出便携模板来源与目录。`source/models` 和 `source/initial_objects.world` 从队内 10 月 30 日分发包导入；`source/classic_world_base.world` 沿用本仓库 Classic 设置。[generate_classic.py](../simulation/worlds/cn_v3_draft/generate_classic.py) 从这些输入生成 Classic 模型与 world，不携带外部绝对路径，也不加载 Fortress 插件。

规则或几何改变时，更新对应模板输入，再运行：

```bash
python3 simulation/worlds/cn_v3_draft/generate_classic.py
```

随后按原构建流程安装，检查资源解析、场地尺寸、道具初态和实际传感器输入，再重新采集匹配的 PCD / 栅格。生成文件不单独手改。旧场地用于复现实验时继续选 `field:=legacy`。

## 本次验证范围（2026-10-31）

已在实际 ROS 2 Humble / ament 环境完成 `rc_field_nav` 的独立 CMake configure、build、install；安装到临时前缀后，实际执行场景选择函数，验证 legacy 默认、V3 默认和用户 world/x/y 覆盖均生效。检查源目录与安装目录的 world / 模型 XML、全部包内 URI、97/79 计数、54 个模型初态及模板一致性，并在另一临时目录重新生成，生成内容逐文件一致。

随后实际进入用户的 `rc27_nav` Docker，执行 `ign gazebo --version` 确认 6.18.0。在容器 `/tmp` 复制本目录，使用独立 `IGN_PARTITION=scu27rc_v3_review_20261031_01`、`ROS_DOMAIN_ID=209`，以无 GUI、初始暂停方式加载 `source/initial_objects.world`。`scene/info` 实际读到 54 个模型：场地 1、红块 20、蓝块 20、塔顶 12、球 1；没有机器人。

仅向该独立 world 发出 `pause:true,multi_step:10`，服务返回 `data:true`，实际 stats 为 `iterations:10`、仿真时间 `.010 s`、`paused:true`，步后仍为 54 个模型。该 server 日志没有错误，SIGINT 正常退出码 0，自己的进程组无残留；未控制或停止已有仿真。原始 scene、stats、日志及 summary 留在容器 `/tmp/scu27rc_v3_review_20261031/validation/`。

新增 `start_fortress_field.sh` 也已在另一独立 partition `scu27rc_v3_review_20261031_02` 实际执行 `-s --iterations 10`，退出码 0、无错误和自身进程组残留；此次没有启动 GUI。

这证明当前 Docker 能真实加载便携 V3 场地并执行短程物理更新。本轮尚未完成 Classic server / GUI、机器人落稳、接触标定、雷达、LIO 或 Nav2 联动验证。场地从 Fortress 移植到 Classic 后仍需重新测量接触；10 步加载不代表道具长期稳定或整机能力。当前使用范围按地面调试处理，跨坡、跨层、携物和完整比赛决策需要各自的实际结果。
