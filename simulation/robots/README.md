# 现有 TR / BR video_v2 模型

这里移入队内已经使用的 `robocon_tr_video_v2` 和 `robocon_br_video_v2`，保留原外形、碰撞、惯性、机构关节和 `/tr_v2`、`/br_v2` 控制话题。TR 为普通轮底盘，BR 保留四个麦轮及 48 个独立被动滚子关节。两车属于此前依据参考视频构建的仿真原型，尺寸是设计值，尚非实车 CAD 或完成标定的比赛机器人。

## 在 V3 场地查看两车

在仓库根目录运行已有双车入口：

```bash
./sim_robots.sh --docker rc27_nav
./sim_robots.sh --docker rc27_nav -s --iterations 1000
```

第二行用于无 GUI 有限步运行。入口加载 [V3 双车 world](../worlds/cn_v3_draft/worlds/robocon_cn_v3_draft_tr_br.world)，共 56 个模型：场地 1、道具 53、BR 1、TR 1。启动器按目录设置资源路径，使用独立 Ignition partition；不会启动 ROS、LIO 或 Nav2。更多场地说明见 [场地 README](../worlds/cn_v3_draft/README.md)。

两车采用收拢关节零位，yaw 均为 π/2。TR 原点为 `(-5.15,-5.116825928,.025100001)`，BR 原点为 `(-4.35,-5.166075008,.030625772)` 米。非对称碰撞包络中心分别位于两个红方启动区中心；BR 滚子最低处比 `base_footprint` 低约 5.526 mm，因此两车使用不同 z。

当前 world 启用 Physics、UserCommands、SceneBroadcaster。模型保留原雷达、IMU、相机描述，但此入口没有启用 Sensors / Imu 系统，不采集这些传感器，也不加载 RGL 插件。底盘、夹具、升降等物理关节及原控制系统仍存在。这里的 Fortress 系统插件不能直接作为 Gazebo Classic 11 插件使用。

## 目录职责

- `models/robocon_{br,tr}_video_v2/`：供 Fortress include 的生成资源，含 `model.sdf`、`model.config` 和标签网格。URI 为包内 `model://`，每车仅一个 8,073 字节的 DAE，无外部图片依赖，其余形状为 primitives。
- `generate_models.py`：便携生成入口，调用两车生成源，再使用 `ign sdf -p` 转成 SDF，并重写网格 URI。
- `source/generate_{br,tr}_video_v2.py`：实际机构与外形生成源。TR 生成器已补回原模型后来加入的橙/蓝指架和金属/橡胶双层垫视觉，使重新生成结果与原模型一致。
- `source/sensor_visuals_v2_20261022.py`：共同传感器外形与 DAE 标签生成源。
- `source/templates/robocon_br_proto.urdf`：BR 生成器所需的实际早期原型输入，包含当时已有修正。
- `source/imported/`：移植前两车原 URDF 和原描述包清单，记录来源。
- `source/generated/`：重新生成的 URDF、标签网格、机构参数、姿态及桥配置，便于阅读和后续接入；桥配置不会由双车展示入口自动启动。
- `source/generate_prototypes_legacy.py`：更早的原型生成来源，保留作追溯；其初始输出与实际模板有后续修正差异，当前入口不调用它覆盖模板。
- `COLCON_IGNORE`：本目录作为仿真资源维护，不由 colcon 发现为额外 ROS 工作空间。

## 重新生成

直接使用已提交的模型只需要 Gazebo Fortress 6。重新生成需要 Python 3、NumPy、PyYAML，以及提供 `ign sdf` 的 Fortress / SDFormat 环境，不需要编译 C++ 或构建 ROS 包：

```bash
python3 simulation/robots/generate_models.py
```

也可用 `--output /另一输出目录` 将生成结果写到独立位置。生成器从自身目录读取源和模板，不依赖原作者路径。没有本机 `ign` 时，可在当前容器离线生成：

```bash
ROBOT_TMP="/tmp/scu27rc_robots_${USER}_$$"
docker cp simulation/robots "rc27_nav:$ROBOT_TMP"
docker exec rc27_nav python3 "$ROBOT_TMP/generate_models.py" --output "$ROBOT_TMP/output"
docker cp "rc27_nav:$ROBOT_TMP/output/models/." simulation/robots/models/
docker cp "rc27_nav:$ROBOT_TMP/output/source/generated/." simulation/robots/source/generated/
```

先修改生成源或模板，再运行入口；不要单改生成的 URDF/SDF。`source/generated` 中 URDF 保留原 `package://` 标识；转换时只在临时输入中改为便携 `model://`，不改变物理参数。

## 原控制话题

这些是原生 Ignition Transport 接口；当前入口没有 ROS bridge。

- `/tr_v2/cmd_vel`、`/br_v2/cmd_vel`：`ignition.msgs.Twist`，原底盘控制接口。
- `/tr_v2/joint/<关节名>/cmd_pos`、`/br_v2/joint/<关节名>/cmd_pos`：`ignition.msgs.Double`，机构位置目标。
- `/tr_v2/sim/joint_states`、`/br_v2/sim/joint_states`：`ignition.msgs.Model`，实测关节状态。
- `/tr_v2/ground_truth/odometry`、`/br_v2/ground_truth/odometry`：`ignition.msgs.Odometry`，各车独立真值。

TR 有 7 个机构位置控制关节，BR 有 12 个，具体名称、初始值与预设姿态见对应 `source/generated/*/config/` 和生成源。原 ROS 桥配置对应 `geometry_msgs/msg/Twist`、`std_msgs/msg/Float64`、`sensor_msgs/msg/JointState`，启用桥接属于单独集成。

## 已验证范围与来源

两车在容器内由上述源重新生成，URDF 元素与移植前原件一致；两份 SDF 通过 `ign sdf -k`。SDFormat 合并固定关节后，BR 为 66 link / 65 joint / 112 collision / 139 visual，48 个滚子关节保留；TR 为 12 link / 11 joint / 82 collision / 92 visual。固定关节合并造成的计数变化是正常 URDF 转换结果。

本轮独立 Fortress 双车场景实际运行 1000 步退出码 0，原生 `scene/info` 确认 56 模型，BR/TR 独立真值可读，加载后未见穿地或翻倒。该短测覆盖加载和初始落地，不代表自主导航、夹取、携物或跨越 V3 的 300 mm 直接台差已完成。

来源为队内此前的 TR/BR 描述包与 `tr_video_v2_generate.py`、`generate_video_v2.py`、`sensor_visuals_v2_20261022.py`、`generate_tr_br_prototypes.py`。原两包 `package.xml` 声明 `Apache-2.0`，导入清单保存在 `source/imported/`；原目录未见单独 LICENSE / NOTICE 文件。模型是视频概念的独立几何实现，未取得或移入视频作者 CAD；视频、赛事规则与场地设计保留各自来源和归属。
