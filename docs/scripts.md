# 一键启动脚本目录

从仓库根目录运行。Fortress 场景入口直接使用已有仿真环境；手工使用 ROS 导航命令前执行 `source env.sh`。首次配置与完整步骤见 [上手与部署](getting_started.md)。

## V3 双机器人：Fortress

| 命令 | 作用 | 前提 |
| --- | --- | --- |
| `./sim_robots.sh` | V3 场地、53 道具与 BR／TR 原型 | 已有 Fortress 6.18，无需 ROS 源码构建 |
| `./sim_robots.sh --docker rc27_nav` | 使用现有容器启动双车场景 | 将运行资源复制到私有临时目录，退出后清理 |
| `./sim_robots.sh -s --iterations 10` | 无 GUI 运行有限物理步 | 同上 |
| `./sim_robots.sh --docker rc27_nav -s --iterations 10` | 容器内无 GUI 短测 | 不改变原容器挂载 |
| `./sim_robots.sh --docker rc27_nav -s --iterations 1000` | 本轮实际执行的 1000 步入口 | 56 模型加载、退出码 0，见 [验证记录](status.md) |
| `IGN_PARTITION=scu27rc_v3_tr_br_demo ./sim_robots.sh` | 在指定独立分区运行双车场景 | 与其他仿真使用不同分区 |
| `./simulation/worlds/cn_v3_draft/start_fortress_field.sh` | 保留的仅场地／道具入口 | 54 个模型，不含机器人 |

`sim_robots.sh` 转调场地目录下的 `start_fortress_robots.sh`，使用 `robocon_cn_v3_draft_tr_br.world`。保留 `/br_v2`、`/tr_v2` 原生控制话题，默认不启 GPU Sensors，不启动 ROS、LIO、Nav2。控制与生成源见 [机器人模型](../simulation/robots/README.md)，容器运行方式和初态见 [地图说明](maps.md)。Ctrl-C 停止本次场景，Docker 模式清理自己的进程组与临时资源。

## 双车同场：Classic（同一个 Gazebo）

| 命令 | 作用 | 说明 |
| --- | --- | --- |
| `./sim_two.sh` | 一个 Gazebo 里两辆车 + **两个 RViz** | 默认开 Gazebo 界面与两个 RViz 窗口 |
| `./sim_two.sh gui:=false` | 只开两个 RViz | 无 Gazebo 界面 |
| `./sim_two.sh field:=cn_v3_draft x1:=-4 y1:=1 x2:=-4 y2:=3` | 换场地与出生点 | 两车位置分开即可 |
| `./sim_teleop.sh --topic /car1/cmd_vel` | 遥控一号车 | 另开终端 |
| `./sim_teleop.sh --topic /car2/cmd_vel` | 遥控二号车 | 另开终端 |

两辆车各自一个 ROS 命名空间，互不干扰：

- `car1`：`/car1/cmd_vel`、`/car1/odom`、`/car1/livox/{lidar,imu}`、`/car1/scan`，TF `car1/odom → car1/base_link`
- `car2`：同上，前缀换成 `car2`
- 两个 RViz 的 Fixed Frame 分别是 `car1/odom`、`car2/odom`（配置在 `rviz/two_car1.rviz`、`rviz/two_car2.rviz`）

实现约束（踩过的坑）：

- 同一个 gzserver 里两套 `gazebo_ros2_control` 的 `controller_manager` 会互相干扰（服务不可达），所以双车用 `kinematic:=true` 的 `planar_move` 运动学底盘，**轮子只做外观不转**；单车仍是 physics + ros2_control。
- Gazebo 插件的 `name` 必须唯一（`imu_plugin_car1` 等），否则 `gazebo_ros_node` 报 "Found multiple nodes with same name" 并让 gzserver 段错误。
- 雷达点云的 frame_id 仍是 `livox_frame`（没给 link 加前缀），所以两个 RViz 里默认只画车体/里程计/激光扫描，不画点云。

## 构建与环境

| 命令 | 作用 |
| --- | --- |
| `./build.sh` | 分阶段构建第三方依赖与根 src 下全部项目包 |
| `./build.sh robot` | 构建真机所需依赖、relocalization 和 mcu_bridge；跳过 rc_field_nav、仿真插件及对照用 fast_lio |
| `./build.sh deps` | 仅构建依赖 |
| `./build.sh nav` | 依赖就绪后，只构建项目工作区 |
| `./third_party/build_deps.sh fast_livo` | 单组件构建；另有 small_gicp／teaser／vikit／livox／fast_lio |
| `source env.sh` | 加载 Humble 与本仓库构建产物；DDS 域默认 42，Gazebo 端口 11355 |

系统 ROS／Nav2、系统库和 Livox-SDK2 需预先安装；Classic 整车仿真另需相应 Gazebo 与控制插件，见 [依赖说明](../third_party/README.md)。换机器、架构或源码路径后重新构建。

## omni4 导航仿真：Classic

| 命令 | 作用 | 常用参数／前提 |
| --- | --- | --- |
| `./sim_map.sh` | 场地、机器人、传感器与 FAST-LIVO2 建图 | `field:=`、`world:=`、`x:=`、`y:=`、`z:=`、`yaw:=`、`gui:=`、`rviz:=` |
| `./sim_teleop.sh` | 键盘发布速度 | 建图／仿真在运行，切到导航前停止遥控 |
| `./sim_save_map.sh [名字]` | 保存先验 PCD、轨迹与二维图 | 保持建图运行；名字是位置参数，默认 `prior` |
| `./sim_nav.sh` | 先验图定位、FAST-LIVO2、Nav2 | `field:=`、`prior_map:=`、`map:=`、`params_file:=` |
| `./sim_nav.sh reloc_impl:=voxel` | 保留的旧体素定位实现 | 需要对应地图，供对照使用 |

`field:=legacy` 为旧场地默认值；国内 V3 征求意见稿场地使用 `field:=cn_v3_draft`。建图和导航保持相同 field 与出生点，换场地后重建配套地图。路径与完整 V3 命令见 [地图与场地更新](maps.md)。

`sim_nav.sh` 优先使用默认 `prior_map.pcd + prior_costmap.yaml`；默认对不存在时，才选择最近一次有成对文件的命名地图。指定数据时显式传入同次保存的 `prior_map` 与 `map`。`./sim_nav.sh --dry-run` 可仅查看选择结果。

## 真机先验图与 Nav2

| 命令 | 作用 | 常用参数／前提 |
| --- | --- | --- |
| `./real_map.sh` | 驱动、抽稀、LIO、轨迹与累积建图 | `rviz:=`、`points_per_frame:=`；接通雷达 |
| `./real_save_map.sh [名字] --imu-height 0.30` | 点云、轨迹、二维图与检查 | 建图保持运行；高度填实测米值 |
| `./real_reloc.sh` | 6-DOF 重定位与 LIO | 默认 `nav2:=false`，可选 `map_path:=`、`rviz:=` |
| `./real_nav.sh` | 6-DOF 重定位、LIO、Nav2 | 成对先验图；`map_path:=`、`map:=`、`params_file:=` |
| `./real_nav.sh nav2:=false` | 只验证定位 | 先观察点云对齐和 `/reloc_status` |
| `./real_nav.sh reloc:=false` | 保留的旧定位流程 | 参数见 `real_voxel_localization.launch.py` |
| `./check_lidar.sh` | 雷达／IMU 频率与时间戳检查 | 驱动运行时执行 |

`real_map.sh` 和 `real_nav.sh` 默认启动自己的驱动。切换流程前停止原入口；需要复用独立驱动时，按入口支持的 `livox_driver:=false` 和现有冲突检查处理。Nav2 输出 `/cmd_vel`，真机底盘的执行接入见 [部署说明](getting_started.md)。

## 小机器人 MCU 位置闭环

```bash
MCU_DRY_RUN=1 ./start_robot.sh   # 先看帧和坐标，不打开串口
./start_robot.sh                 # 位姿／目标位置帧发到 MCU
RVIZ=1 ./start_robot.sh          # 机器人有显示环境时启动 RViz
./rviz_robot.sh                  # 开发机看 odom 系
```

根 `start_robot.sh` 转调 `robot/start_robot.sh`。该入口使用本仓库驱动配置与 `third_party/install` 的 FAST-LIVO2；LIO 订阅原始 `/livox/imu`，输出 `odom→base_link`，不另起 IMU 翻转节点。

- `DOMAIN=0` 可为小机器人脚本和 `rviz_robot.sh` 指定 DDS 域；通用环境使用 `RC_DOMAIN_ID`。
- `MCU_PORT=/dev/...` 可覆盖小机器人串口设备；独立串口包读取自己的 YAML。
- `MCU_DRY_RUN=1` 控制串口空跑；`RVIZ=1` 控制机器人上 RViz。
- 此流程由 MCU 做位置闭环，不启动 Nav2 或全局重定位。RViz Fixed Frame 采用 `odom`。

## 串口单包联调与部署

```bash
source env.sh
ros2 launch mcu_bridge mcu_bridge.launch.py dry_run:=true
python3 src/mcu_bridge/test/test_frame.py
```

正常串口启动去掉 `dry_run:=true`。设备、坐标符号、频率与可选速度帧见 [mcu_bridge README](../src/mcu_bridge/README.md) 和 [配置](../src/mcu_bridge/config/mcu_bridge.yaml)。

`tools/deploy_mcu_bridge.sh` 是单包 rsync＋远端构建工具，默认目标和 prefix chain 行为写在脚本里。部署前查看目标配置，完整仓库部署用 [上手与部署](getting_started.md) 的构建方法。

串口保持单写者；`mcu_sniff` 会读取串口字节，使用时停止实际通信。人工遥控与自动导航切换时先停止原速度来源。

## 停止与定位状态

手工启动的流程用 Ctrl-C 停止。若目标机已安装 `robot-nav.service` 用户服务，先用 `systemctl --user stop robot-nav.service` 停服务，再手工联调；服务是否部署以目标机为准。

`reloc_bridge` 在 `/reloc_status` 输出 BOOTSTRAP／SEARCHING／APPLIED／TRACKING；首次定位前也可能有 identity `map→odom`。导航观察同时看状态、点云与目标结果。历史退出日志说明保存在 [项目当前情况](status.md)。

## Decide

当前 `decision/` 是 5 份设计文档。运行入口应在节点与 launch 实现并验证后加入本目录；设计内容从 [decision/README](../decision/README.md) 阅读。
