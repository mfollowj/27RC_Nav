# 工程组织与比赛项目参考

## 高 star 工程组织参考

以下 star 数量于 2026-10-31 从 GitHub 官方仓库 API 读取，布局对照来自原仓库源码。

| 项目 | Stars | 本仓库采用的组织方式 |
| --- | ---: | --- |
| [navigation2 官方元数据](https://api.github.com/repos/ros-navigation/navigation2) | 4,966 | 算法按包分工；bringup 集中安装 launch、params、maps、rviz。[安装规则](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/CMakeLists.txt) |
| [pb2025_sentry_nav 官方元数据](https://api.github.com/repos/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) | 457 | 启动、地图与 real/sim 参数职责明确。[bringup 目录](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/tree/main/pb2025_nav_bringup) |
| [RoboRTS 官方元数据](https://api.github.com/repos/RoboMaster/RoboRTS) | 898 | 定位、规划、通信、决策分工清楚，bringup 保留操作资源。[目录](https://github.com/RoboMaster/RoboRTS/tree/icra2019)；这是历史 ROS 1 工程 |
| [FAST_LIO 官方元数据](https://api.github.com/repos/hku-mars/FAST_LIO) | 5,324 | 算法包内部区分 src、include、config、launch，并保留上游项目边界。[源码](https://github.com/hku-mars/FAST_LIO)；此处参考其包内组织 |

这些项目并不使用完全相同的目录名：Nav2 使用 `params/`，pb2025 使用 `config/`；它们的共性是职责清楚、资源安装和入口引用一致。

本仓库据此实际提升根 `src/`，把真机资源放到 `robot/`，共用配置放到 `config/`，场地放到 `simulation/`，第三方源码放到 `third_party/`。保留真机单独构建的边界和已有 ROS 接口；迁移清单见 [命名与迁移](naming.md)。ROS 1 示例只作布局参考，Nav2 主分支的新接口也不能直接代入当前 Humble。

## 近期比赛场景样本

下面 10 项按赛季、比赛任务与导航场景选择，用于对照具体实现；它们不是按 star 数量筛选的工程标杆名单。pb2025 同时属于上方工程参考，其余项目按实际源码和验证范围判断适用性。

调研日期：2026-10-31。对象为 4 个 RC 与 6 个 RM 公开仓库；已通过 GitHub 连接读取默认分支 README（缺 README 时读项目总览）、递归目录、主要 launch 与配置。下文描述的是公开实现与说明，没有编译、运行或验证这些项目的比赛表现。最近提交日期是默认分支 HEAD 的 committer 时间，不能作为导航能力成熟度指标；部分最近提交仅更新文档或工具。

当前最适合借鉴的是：RC 任务编排看 RC_2026、Robocon_R2 和 robocon26，导航包边界看 gn10-navigation，Nav2 工程启动与参数说明看 pb2025_sentry_nav，复杂项目的源码分类与分级 README 看 SentryNav2026_XDU。Aurora 的通行方向标注适合研究高低台与坡道语义，但其 ROS 2 Jazzy 自研规划链不是 SCU27RC_Nav 的直接替换件。

### 1. RC：potato484/RC_2026

最近默认分支提交：[2026-08-29 UTC](https://github.com/potato484/RC_2026/commit/d32c1b9fb94da305c4c0e14047a345734a98e732)。比赛背景：ABU ROBOCON 2026 R2；ROS 2 Humble。

- 组织：`src/rc26_bringup` 装配运行模式；`rc26_decision` 负责行为树；驱动、Point-LIO、odom 接口、外参、MCU transport、视觉分别成包；红蓝方配置位于 bringup/config。
- 当前导航：README 与独立右转入口均使用 Point-LIO → 统一 /odom → odom 相对位移／转向闭环；当前默认任务并不依赖 Nav2 地图规划。定位与点云转换工具仍保留，不能因仓库有这些包就写成默认地图导航链。
- 可借鉴：明确“最短可运行链”和各包输入输出，提供单项动作入口，再进入完整比赛树。对 RC，抓取、上台、放置等任务比 RM 裁判消息更相关。
- 范围：相对闭环适合指定动作，不能直接替代需要全场避障与全局重定位的地图导航。

依据：[项目说明](https://github.com/potato484/RC_2026/blob/d32c1b9fb94da305c4c0e14047a345734a98e732/README.md)；[独立导航入口](https://github.com/potato484/RC_2026/blob/d32c1b9fb94da305c4c0e14047a345734a98e732/src/rc26_bringup/launch/odom_right_turn_nav.launch.py)；[实际动作树](https://github.com/potato484/RC_2026/blob/d32c1b9fb94da305c4c0e14047a345734a98e732/src/rc26_decision/behavior_trees/odom_right_turn_nav_tree.xml)。

### 2. RC：hiinguyen/Robocon_R2

最近默认分支提交：[2026-07-29 UTC](https://github.com/hiinguyen/Robocon_R2/commit/1bcb4ee144b2d097b7d43fda63787ba3d4e5d3dd)。比赛背景：ABU ROBOCON 2026，AK60 R2；ROS 2、py_trees、ros2_control。

- 组织：机器人 bringup、description、MoveIt、控制驱动、硬件 IO、navigation、slam 分包；`PROJECT_OVERVIEW.md` 提供从包到 ROS Action 与任务树的索引，仓库另有 code tour。
- 当前导航：`my_robot_navigation` 复用 nav2_bringup；参数中为 AMCL、Navfn、DWB。任务树另编排相对导航、靠墙、攀爬、机械臂等动作。
- 地图：导航包与 bringup 都存在 `my_map.yaml`，两份同名配置值得对照；这种重复不适合照搬到 V3 地图管理。
- 可借鉴：新人总览直接连接“比赛动作 → Action → 源码 → launch”；把导航与机构动作通过任务层组合。

依据：[项目总览](https://github.com/hiinguyen/Robocon_R2/blob/1bcb4ee144b2d097b7d43fda63787ba3d4e5d3dd/PROJECT_OVERVIEW.md)；[Nav2 入口](https://github.com/hiinguyen/Robocon_R2/blob/1bcb4ee144b2d097b7d43fda63787ba3d4e5d3dd/src/my_robot_navigation/launch/nav_bringup.launch.py)；[Nav2 配置](https://github.com/hiinguyen/Robocon_R2/blob/1bcb4ee144b2d097b7d43fda63787ba3d4e5d3dd/src/my_robot_navigation/config/nav2_params.yaml)；[任务树源码](https://github.com/hiinguyen/Robocon_R2/tree/1bcb4ee144b2d097b7d43fda63787ba3d4e5d3dd/src/ak60_bringup/scripts/r2_bt)。

### 3. RC：yashdoke7/robocon26

最近默认分支提交：[2026-07-07 UTC](https://github.com/yashdoke7/robocon26/commit/047a7817761a938edb6c061eb38f9d7cf154df65)。比赛背景：ABU ROBOCON 2026，Team Vulcans R2。

- 组织：`Deployment/` 是当前 Python 运行实现，按 common、zone1、zone2、zone3、firmware、sim、tests 划分；`Zone1/Zone2/Zone3` 还包含研究、训练与资产。当前运行入口不是统一 ROS 2 launch 包。
- 导航语义：Zone 2 使用 4×3 格点与高度配置，任务包含通行、抓取和上／下台阶；红蓝方由任务配置处理。总比赛入口顺序执行多个区段。
- 可借鉴：区分“运行实现”和“研究／训练资产”，将每个比赛区域的几何与动作参数集中到可定位的配置。
- 范围：其研究原型、训练代码和运行实现必须分别阅读；README 中的“proven”等用语没有在本次调研中得到运行证据。

依据：[根说明](https://github.com/yashdoke7/robocon26/blob/047a7817761a938edb6c061eb38f9d7cf154df65/README.md)；[当前运行目录说明](https://github.com/yashdoke7/robocon26/blob/047a7817761a938edb6c061eb38f9d7cf154df65/Deployment/README.md)；[整场入口](https://github.com/yashdoke7/robocon26/blob/047a7817761a938edb6c061eb38f9d7cf154df65/Deployment/run_match.py)；[格点与高度配置](https://github.com/yashdoke7/robocon26/blob/047a7817761a938edb6c061eb38f9d7cf154df65/Deployment/zone2/config.py)。

### 4. RC：tmcit-ararobo-2026a/gn10-navigation

最近默认分支提交：[2026-10-31 UTC](https://github.com/tmcit-ararobo-2026a/gn10-navigation/commit/94d61a62969baa5c86290d771f848c69eafe62db)。README 说明面向 NHK ロボコン 2026 等自动走行需求；ROS 2 Humble。它与 ABU ROBOCON 2026 不是同一场地图。

- 组织：独立 ROS 包，四个节点组分别负责目标管理、代价地图生成、路径规划、路径跟踪；`launch/navigation.launch.py` 集中组合，config 按职责分文件。
- 地图与规划：JSON 场地定义生成 OccupancyGrid；8 邻域 A* 和 B-spline 路径；跟踪器使用 Pure Pursuit。当前定位 TF 来自包外，文档明确写出供应关系。
- 可借鉴：把一个导航包的输入、输出、参数、外部依赖写成一条可追踪数据流；地图源与运行参数分开。
- 具体限制：其配置主动采用 0.45 m 不可侵半径，README 明示小于约 0.596 m 外接半径，中心点碰撞检查在部分姿态允许机体接触。不得把该配置当成 SCU 机器人尺寸或安全余量模板。

依据：[包说明与接口](https://github.com/tmcit-ararobo-2026a/gn10-navigation/blob/94d61a62969baa5c86290d771f848c69eafe62db/README.md)；[导航入口](https://github.com/tmcit-ararobo-2026a/gn10-navigation/blob/94d61a62969baa5c86290d771f848c69eafe62db/launch/navigation.launch.py)；[路径规划实现](https://github.com/tmcit-ararobo-2026a/gn10-navigation/blob/94d61a62969baa5c86290d771f848c69eafe62db/src/path_planner_node.cpp)；[场地膨胀参数](https://github.com/tmcit-ararobo-2026a/gn10-navigation/blob/94d61a62969baa5c86290d771f848c69eafe62db/config/costmap_params.yaml)。

### 5. RM：SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav

最近默认分支提交：[2026-10-26 UTC](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/commit/8177b152fb8b70de0f5c7b7160bdfd386d4877ed)，该提交为工具自动更新；比赛背景为 RoboMaster 2025。ROS 2 Humble。

- 组织：独立算法／接口包与 `pb2025_nav_bringup`；仿真与实车各有入口，参数按 reality／simulation 分开。根 README 有目录职责、环境、编译、运行、启动参数和 TF 说明。
- 当前参数：Point-LIO、small_gicp、地形处理；所读实车配置选择 ThetaStar 全局规划和 OmniPidPursuit 跟踪。不要只写“Nav2 默认规划器”而漏掉当前 YAML 的实际插件。
- 地图：`world` 关联栅格与先验点云路径；仿真地图包含多个年份。大先验点云另行下载，说明把缺失资产的获取入口写清楚。
- 可借鉴：建图、定位、导航和仿真实车使用相同语义的参数；公开 TF 变换所有者。自旋虚拟坐标系是否需要由 SCU 底盘和传感器安装决定。

依据：[README](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/8177b152fb8b70de0f5c7b7160bdfd386d4877ed/README.md)；[实车装配入口](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/8177b152fb8b70de0f5c7b7160bdfd386d4877ed/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py)；[实际导航参数](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/8177b152fb8b70de0f5c7b7160bdfd386d4877ed/pb2025_nav_bringup/config/reality/nav2_params.yaml)。

### 6. RM：XDU-IRobot/SentryNav2026_XDU

最近默认分支提交：[2026-08-01 UTC](https://github.com/XDU-IRobot/SentryNav2026_XDU/commit/8297bab6eda13e30ed19e770862288a5c2313274)。RoboMaster 2026；ROS 2 Humble。

- 组织：`src/perception`、`navigation`、`decision`、`communication`、`bringup`、`tools`；根 README 链接模块 README，明确哪些通信包当前使用、哪些已废弃。
- 当前参数：Super-LIO／Lightning-LM，Nav2 的 SmacPlanner2D 与 ElasticPlanner 可选；控制器含 MPPI wrapper 与分模式 MPPI。行为树决策与导航恢复树是不同层，不应在文档混为同一棵树。
- 地图与部署：地图资产和比赛数据目录分散于根与 bringup；中央启动器读取顺序配置。可以借鉴启动职责与阅读索引，地图目录仍需结合自身流程整理。
- 可借鉴：大型 src 按功能域归类，每个入口可一路定位到模块说明和参数。README 明确它有双 yaw、越线、裁判通信等 RM 专用设计，不适合原样作为 RC 新人模板。

依据：[系统与目录说明](https://github.com/XDU-IRobot/SentryNav2026_XDU/blob/8297bab6eda13e30ed19e770862288a5c2313274/README.md)；[中央启动器](https://github.com/XDU-IRobot/SentryNav2026_XDU/blob/8297bab6eda13e30ed19e770862288a5c2313274/src/bringup/launch/start.launch.py)；[当前导航参数](https://github.com/XDU-IRobot/SentryNav2026_XDU/blob/8297bab6eda13e30ed19e770862288a5c2313274/src/bringup/config/nav2_params.yaml)；[任务行为树说明](https://github.com/XDU-IRobot/SentryNav2026_XDU/blob/8297bab6eda13e30ed19e770862288a5c2313274/src/decision/sentry_nav_bt/README.md)。

### 7. RM：Aurora-UJS/sentry_nav_26

最近默认分支提交：[2026-08-23 UTC](https://github.com/Aurora-UJS/sentry_nav_26/commit/0da21120a58ef8039df76de1deca8f92d9ae0c4e)。RoboMaster 哨兵方向；ROS 2 Jazzy。

- 组织：`plan_env`、`path_searching`、global／local planner、bringup、仿真、RViz 标注插件分包；这是一条自研规划链，不是 Nav2 参数配置的变体。
- README 描述 JPS 全局规划、Kinodynamic A*／MINCO 局部轨迹、MPC 跟踪；当前 launch 确实组合仿真、LIO、全局与局部规划器。在线地图参数使用 odom 坐标系，仿真入口发布静态 map→odom。
- 通行语义：PGM 之外有 `.trav.yaml`，表达 free／obstacle／oneway；适合启发高低台只能单向通过的表达。它没有证明普通单层栅格已经解决重叠楼层或任意多层地图。
- 文档一致性问题：README 说默认标注为空且关闭，当前 `sim_test.launch.py` 向两个 planner 注入同一份 `rmuc_2025.trav.yaml`，该文件实际含一个 oneway polygon。独立 planner 的默认空路径与整机入口的默认注入需要分别说明。

依据：[算法与说明](https://github.com/Aurora-UJS/sentry_nav_26/blob/0da21120a58ef8039df76de1deca8f92d9ae0c4e/README.md)；[当前仿真启动链](https://github.com/Aurora-UJS/sentry_nav_26/blob/0da21120a58ef8039df76de1deca8f92d9ae0c4e/src/sentry_bringup/launch/sim_test.launch.py)；[当前方向标注](https://github.com/Aurora-UJS/sentry_nav_26/blob/0da21120a58ef8039df76de1deca8f92d9ae0c4e/src/sentry_bringup/map/rmuc_2025.trav.yaml)；[全局规划器参数注入](https://github.com/Aurora-UJS/sentry_nav_26/blob/0da21120a58ef8039df76de1deca8f92d9ae0c4e/src/sentry_global_planner/launch/global_planner.launch.py)。

### 8. RM：Boombroke/Sentry26

最近默认分支提交：[2026-06-03 UTC](https://github.com/Boombroke/Sentry26/commit/9f3ae17627cef5ca1f102a27668f8b5e78a2cdb1)。RoboMaster 2026；ROS 2 Jazzy，差速轮足底盘。

- 组织：decision、motion manager、nav、bringup、description、serial、simulator；导航 bringup 同样区分仿真和实车参数。
- 当前参数：Point-LIO、small_gicp；实车配置选择 MPPI DiffDrive，Nav2 基座为 base_footprint；双 Mid360 的输入与 odom 桥接在文档分开交代。
- 可借鉴：TF 树、速度指令链和实车／仿真配置的说明；明确哪个传感器提供 LIO 的 IMU。
- 范围：README 明示部分全场验证仍待完成；Route Graph、Lattice／MINCO 等是后续路线图，不能写成当前已实现能力。差速模型也不能直接复制到全向 RC 底盘。

依据：[系统与实现范围说明](https://github.com/Boombroke/Sentry26/blob/9f3ae17627cef5ca1f102a27668f8b5e78a2cdb1/README.md)；[实车启动](https://github.com/Boombroke/Sentry26/blob/9f3ae17627cef5ca1f102a27668f8b5e78a2cdb1/src/sentry_nav_bringup/launch/rm_navigation_reality_launch.py)；[当前实车参数](https://github.com/Boombroke/Sentry26/blob/9f3ae17627cef5ca1f102a27668f8b5e78a2cdb1/src/sentry_nav_bringup/config/reality/nav2_params.yaml)。

### 9. RM：qkrt-rm/qkrt-nav

最近默认分支提交：[2026-10-26 UTC](https://github.com/qkrt-rm/qkrt-nav/commit/75d6f614579d15230e2c81477d17ff23389ddbc6)。RoboMaster／ARC 哨兵场景；ROS 2 Humble。

- 组织：bringup、communication、description、drivers、localization、navigation、vision 分包；Nav2 的 planner／controller／behavior／smoother／BT 参数分开存放。
- 当前参数：SmacPlanner2D、OmniPursuitController、AMCL／SLAM Toolbox、EKF；场地地图另有 keepout mask，导航入口可启停禁行区。
- 可借鉴：把机器人 TF、速度坐标变换、真实传感器、定位、地图、任务依赖列清楚；文档将已知问题直接关联到可观察症状。
- 范围：README 列有旋转与去畸变问题、未完成的符号核验和端到端导航工作，不足以支持“成熟稳定产品”的结论。其地图目录还有多份测试地图，应结合各 launch 默认项甄别。

依据：[实现说明与已知问题](https://github.com/qkrt-rm/qkrt-nav/blob/75d6f614579d15230e2c81477d17ff23389ddbc6/README.md)；[整机入口](https://github.com/qkrt-rm/qkrt-nav/blob/75d6f614579d15230e2c81477d17ff23389ddbc6/src/sentry_bringup/launch/real_robot.launch.py)；[规划参数](https://github.com/qkrt-rm/qkrt-nav/blob/75d6f614579d15230e2c81477d17ff23389ddbc6/src/sentry_navigation/config/planner_server.yaml)；[控制与禁行层参数](https://github.com/qkrt-rm/qkrt-nav/blob/75d6f614579d15230e2c81477d17ff23389ddbc6/src/sentry_navigation/config/controller_server.yaml)。

### 10. RM：qza36/COD_NAV

最近默认分支提交：[2025-08-02 UTC](https://github.com/qza36/COD_NAV/commit/54dc95af42cb5e3908bedba8005ced1f8f71d8e8)。RoboMaster 2025 COD 哨兵；ROS 2 Humble。

- 组织：FAST_LIO、small_gicp、地面分割、点云转激光、Nav2 插件与 `nav_bringup`，另有仿真模块。
- 地图：PCD 用于重定位，再通过 pcd2pgm 生成二维导航图；launch 包中包含多套地图与参数。
- 文档一致性问题：README 说局部规划采用 MPPI；当前 `nav_bringup/params/nav2_params.yaml` 的有效 FollowPath 为 OmniPidPursuitController，MPPI 段被注释，全局为 SmacPlannerHybrid。多配置存在时必须说明实际入口加载哪一份，不能从 README 或文件名推断运行选择。
- 可借鉴：记录 PCD→PGM 转换和雷达安装参数入口；不要照搬多个近似参数文件却缺少选择说明的组织方式。

依据：[仓库说明](https://github.com/qza36/COD_NAV/blob/54dc95af42cb5e3908bedba8005ced1f8f71d8e8/README.md)；[传感器与定位装配](https://github.com/qza36/COD_NAV/blob/54dc95af42cb5e3908bedba8005ced1f8f71d8e8/nav_bringup/launch/bringup.launch.py)；[实际插件配置](https://github.com/qza36/COD_NAV/blob/54dc95af42cb5e3908bedba8005ced1f8f71d8e8/nav_bringup/params/nav2_params.yaml)。

## 对 SCU27RC_Nav 的整理建议

这些组织方式用于本仓库的阅读入口、源码索引与后续改造；当前路径见 [源码与配置索引](source_index.md)。

1. **先写阅读入口。** 根 README 说明机器人、比赛、当前可运行能力、最短启动步骤；模块索引把源码目录、启动入口、参数、地图和维护职责对应起来。新人可沿“README → 架构与数据流 → 仿真 → 定位 → 导航 → 比赛任务 → 部署与排障”阅读。
2. **src 分类应跟现有包走。** 驱动／定位／导航／任务／通信／整机启动是职责，不必因为参考项目目录漂亮就批量改包名或搬文件。先整理文档索引，确认当前 launch、ament 安装规则和路径引用，再做必要调整。
3. **V3 地图要是一组相互匹配的资源。** 查清其 Gazebo/SDF 场地源、栅格图与 YAML、PCD、坐标原点／分辨率、任务点、红蓝变换和通行语义分别由谁生成、由谁加载；写明更新哪一个源、按什么已有命令派生。外部项目的 2026 地图不能充当本队 V3 的来源证据。
4. **高低台先核对实际场地需求。** 高程、台阶方向、跨区连接、机构动作与任务区状态应按本队 V3 场地表达；单张 PGM 只能表达二维占据，oneway 仅是方向约束，也不等于通用多层地图方案。
5. **TF 和控制链比选算法名字更先要说清。** 为 map→odom、odom→base、base→sensor 标出发布者；说明速度在哪个坐标系、MCU 消费哪个话题。RM 自旋虚拟坐标系、双 yaw 和裁判系统协议只有本队存在对应需求时才考虑。
6. **任务层与导航层分开说明。** Nav2 恢复树控制“如何到达”，RC 任务树／状态机控制“先去哪、何时爬台、何时抓取”；让目标失败、取消、机构完成与人工接管能在文档中顺着链路找到实现。
7. **实测与计划在能力清单中分别表述。** 每项能力链接当前实现和本队已有运行记录；文档更新、外部 README 宣称和路线图不能替代本队仿真／实车结果。现阶段优先保存已有证据，不新加 hash、冻结 contract、baseline 或 gate。

## 历史参考位置

PolarisXQ/SCURM_SentryNavigation 可作为本队已有依赖／历史来源进一步核对，不列入上述 10 个近期样本；没有在本次调研中将它的年份、来源或成熟度补写成事实。
