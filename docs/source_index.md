# 源码与配置索引

按“入口 → launch → 参数 → 实现”查找。以下路径均相对仓库根目录；`build/`、`install/` 是生成产物，修改后通过构建更新。

## 本项目的 4 个 ROS 包

根目录是 colcon 工作区，项目实现位于 `src/`，4 个 ROS 包按实际职责分工。包名与节点接口保留，旧新路径见 [命名与迁移](naming.md)。

| 功能域 | 包名与目录 | 主要内容 |
| --- | --- | --- |
| 仿真与导航装配 | [rc_field_nav](../src/rc_field_nav) | launch、URDF／Xacro、Nav2 YAML、底盘与地图工具 |
| 全局重定位 | [relocalization](../src/relocalization) | 当前主要节点 `teaser_gicp_node`；保留兼容 TF 发布器 |
| 仿真传感器 | [ros2_livox_simulation](../src/ros2_livox_simulation) | Gazebo Classic Mid-360 插件；目录名与包名一致 |
| 电控通信 | [mcu_bridge](../src/mcu_bridge) | 位姿／目标串口帧、可选速度帧、协议测试 |

`relocalization/src/` 只保留 `teaser_gicp_node`、`transform_publisher` 两个当前目标。旧 ICP、SAC-IA、small_gicp、TEASER 实现与旧 launch 已归入 [legacy](../src/relocalization/legacy/)，不参与默认构建或安装；使用说明见 [包 README](../src/relocalization/README.md)。

## 按修改任务定位

| 想改什么 | 先改哪里 | 关联入口与验证 |
| --- | --- | --- |
| V3 场地几何、设施或出生点 | [地图与场地更新](maps.md) 指定的生成源；场地资产在 `simulation/worlds/` | 显式选场地，查看几何与出生点，再重建配套先验图 |
| 机器人尺寸、轮径、质量、雷达安装 | [rc_field_nav/urdf](../src/rc_field_nav/urdf)；底盘实现 [omni_drive.py](../src/rc_field_nav/scripts/omni_drive.py) | 确认碰撞体、TF 与轮速方向；同步相应 Nav2 footprint |
| 真机 LiDAR–IMU 标定、LIO 滤波与地图参数 | [config/livo_mid360.yaml](../config/livo_mid360.yaml) | real／sim 共用；平台差异由各 launch 覆盖 |
| 雷达网络与发布配置 | [mid360_config.json](../robot/config/mid360_config.json)、[real_livox_driver.launch.py](../robot/launch/real_livox_driver.launch.py) | `host_ip`／`lidar_ip` 可覆盖，运行 `check_lidar.sh` 看真实数据 |
| 点云抽稀、距离与高度过滤 | [livox_preprocess.py](../robot/scripts/livox_preprocess.py)，以及调用它的 launch 参数 | 检查原始／过滤话题频率与保留点数 |
| 重定位搜索、配准或消息输出 | [teaser_gicp_node.cpp](../src/relocalization/src/teaser_gicp_node.cpp) | `map_voxel`、`teaser_inlier` 等由启动入口传入 |
| 采纳定位、更新 map→odom、触发 LIO 恢复 | [reloc_bridge.py](../robot/scripts/reloc_bridge.py) | 对照 `/icp_result`、`/relocalization_result`、`/reloc_status` 与 TF |
| 仿真路径规划、速度、代价地图 | [nav2_omni_params.yaml](../src/rc_field_nav/config/nav2_omni_params.yaml) | `./sim_nav.sh params_file:=...`；短目标与避障场景 |
| 真机路径规划、速度、代价地图 | [nav2_real.yaml](../robot/config/nav2_real.yaml) | `./real_nav.sh params_file:=...`；先确认底盘速度链路 |
| 串口设备、缩放、坐标或帧协议 | [mcu_bridge.yaml](../src/mcu_bridge/config/mcu_bridge.yaml)、[frame.py](../src/mcu_bridge/mcu_bridge/frame.py)、[position_bridge_node.py](../src/mcu_bridge/mcu_bridge/position_bridge_node.py) | 先运行现有帧测试与 `dry_run`，再与电控联调 |
| 环境与构建 | [env.sh](../env.sh)、[build.sh](../build.sh)、[third_party/build_deps.sh](../third_party/build_deps.sh) | 确认包解析位置，再构建受影响包 |
| 比赛任务、阶段与机构动作 | [Decide](../decision/README.md) | 先补对应设计与接口，再新增实现；当前为 5 份 Markdown |

## 各入口加载什么

| 根脚本 | 主要装配文件 | 作用 |
| --- | --- | --- |
| [sim_map.sh](../sim_map.sh) | [sim_mapping.launch.py](../src/rc_field_nav/launch/sim_mapping.launch.py) → [sim.launch.py](../src/rc_field_nav/launch/sim.launch.py) | 场地、机器人、传感器、LIO 与建图 |
| [sim_nav.sh](../sim_nav.sh) | [sim_navigation.launch.py](../src/rc_field_nav/launch/sim_navigation.launch.py) → [nav2_only.launch.py](../src/rc_field_nav/launch/nav2_only.launch.py) | 先验图定位、LIO、Nav2；有重定位实现选择 |
| [real_map.sh](../real_map.sh) | [real_mapping.launch.py](../robot/launch/real_mapping.launch.py) | 雷达、点云处理、LIO、轨迹与点云累积 |
| [real_nav.sh](../real_nav.sh)、[real_reloc.sh](../real_reloc.sh) | [real_relocalization.launch.py](../robot/launch/real_relocalization.launch.py) → [real_nav2_only.launch.py](../robot/launch/real_nav2_only.launch.py) | 默认 6-DOF 重定位；`real_reloc.sh` 默认不开 Nav2 |
| `real_nav.sh reloc:=false` | [real_voxel_localization.launch.py](../robot/launch/real_voxel_localization.launch.py) | 保留的旧体素定位工作流 |
| [start_robot.sh](../start_robot.sh) | [robot/start_robot.sh](../robot/start_robot.sh) | 小机器人驱动、LIO 与 MCU 位置闭环接入 |

真机与仿真都加载 [livo_mid360.yaml](../config/livo_mid360.yaml) 和 [camera_pinhole.yaml](../config/camera_pinhole.yaml)。当前 `img_en=0`，相机文件用于参数声明。仿真的 LiDAR–IMU 外参和 `use_sim_time` 在 launch 中覆盖，调仿真时先查覆盖值。

## 机器人与传感器参数

- [omni4_body.xacro](../src/rc_field_nav/urdf/omni4_body.xacro)：车体、轮子、质量与碰撞几何。
- [omni4_field.urdf.xacro](../src/rc_field_nav/urdf/omni4_field.urdf.xacro)：机器人组合、传感器安装与 Gazebo 接入。
- [lidar_livox_mid360.xacro](../src/rc_field_nav/urdf/lidar_livox_mid360.xacro)：仿真雷达参数。
- [omni4_controllers.yaml](../src/rc_field_nav/config/omni4_controllers.yaml)：轮速控制器。
- [pointcloud_to_laserscan.yaml](../src/rc_field_nav/config/pointcloud_to_laserscan.yaml)：感知数据转换。

LiDAR–IMU 的 `extrinsic_T/R` 与 base_link→LiDAR 的安装 TF 表达不同关系。更换雷达安装位置时，分别确认传感器内部标定和车体安装变换。当前 YAML 使用原始 IMU 与单位旋转矩阵；已有坐标补偿应结合实车测量整体核对。

## 定位与导航接口

| 数据或变换 | 来源 | 使用者／含义 |
| --- | --- | --- |
| `/livox/lidar`、`/livox/imu` | 真机驱动或 Gazebo 插件 | LiDAR CustomMsg 与原始 IMU |
| `/livox/lidar_filtered` | `livox_preprocess.py` | FAST-LIVO2 输入 |
| `/aft_mapped_to_init`、`/cloud_registered` | FAST-LIVO2 | 局部里程计与配准点云；串口桥使用该里程计 |
| `/icp_result` | `teaser_gicp_node` | 配准结果交给 bridge 处理 |
| `/relocalization_result`、`/reloc_status` | `reloc_bridge.py` | LIO 重定位输入与运行状态 |
| `map→odom` | 导航入口中的 bridge | 全局定位修正；首次定位前可为 identity |
| `odom→base_link` | LIO 工作流中的 FAST-LIVO2 | `sim.launch.py` 单独使用时也支持轮速 TF，LIO 入口会关闭重复发布 |
| `base_link→livox_frame` | 真机静态 TF 或仿真机器人模型 | 传感器安装变换 |
| `/cmd_vel` | Nav2 或当前选择的遥控入口 | 仿真底盘／接入的真机速度消费者 |

Nav2 参数分别位于真机与仿真 YAML，两边当前选用 SmacPlanner2D 与 MPPI Omni，odom 话题对齐 `/aft_mapped_to_init`。footprint、限速、膨胀半径和传感器源按实际平台核对。当前 LIO 输出的 twist 尚待补充／验证，具体见 [项目当前情况](status.md)。

`/reloc_status` 的现有模式包括 BOOTSTRAP、SEARCHING、APPLIED、TRACKING。观察定位时同时检查状态、点云和 TF；仅有 map 帧不能说明首次重定位成功。

## 地图、依赖与历史资料

- 地图及派生文件：统一查 [地图与场地更新](maps.md)，按来源重新生成。
- 第三方实现：查 [Deps 清单](../third_party/README.md)；FAST-LIVO2 的本地改动见 [补丁说明](../robot/patches/README.md)。
- 历史联调：`archive/legacy_runs/` 与 [项目当前情况](status.md)；一次性脚本不能自动代表当前默认入口。
- 决策设计：`decision/*.md` 中的计划包与 action 应在实现后再加入运行文档。
