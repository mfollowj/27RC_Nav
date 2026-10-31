# 上手与部署

所有命令从仓库根目录运行；每个手工执行 `ros2` 的终端先 `source env.sh`。根目录启动脚本会自行加载环境。

## 1. 准备系统与依赖

目标系统为 Ubuntu 22.04、ROS 2 Humble；整车仿真插件面向 Gazebo Classic 11。真机模式可跳过两个仿真包，使用 `JOBS=1 ./build.sh robot`。

本次实际检查的 Docker 容器 `rc27_nav` 已安装 **Gazebo Fortress 6.18.0**。它可以加载 V3 场地资源；仓库的 Classic 整车雷达插件仍需要相应的 Classic 环境。Fortress 场地入口和 Docker 验证结果见 [地图与场地更新](maps.md)。Gazebo Harmonic／ROS 2 Jazzy 也使用不同的插件与启动环境。

系统应提供这些依赖，具体分层见 [third_party/README](../third_party/README.md)：

- ROS 2 Humble、colcon、ament、Python 3 和 C++ 编译工具。
- Nav2、Gazebo ROS、gazebo_ros2_control、ros2_controllers、xacro、robot_state_publisher、pointcloud_to_laserscan。
- PCL、Eigen、OpenMP、OpenCV、Sophus、fmt、cv_bridge、image_transport，以及 Python 的 numpy、PyYAML、Pillow 和 `python3-serial`。对应系统包包括 `libfmt-dev`、`python3-numpy`、`python3-yaml`、`python3-pil`。
- **Livox-SDK2**：在目标系统编译安装，驱动构建时会链接它；源码目录 `third_party/livox_ros_driver2` 不包含已安装的 SDK。

第三方算法与驱动源码在 `third_party/`，TEASER++ 首次 configure 默认从 GitHub 获取 `pmc`、`tinyply`、`spectra`。离线构建时先准备这三份源码及上述系统包，按 Deps 文档放到对应 `third_party/` 目录。

## 2. 获取源码并构建

```bash
cd ~
git clone https://github.com/SCU-Pangolin/SCU27RC_Nav.git
cd SCU27RC_Nav
./build.sh
source env.sh
```

`build.sh` 先构建依赖，再构建本项目的 4 个 ROS 包。仅需真机链路时使用 `./build.sh robot`，只构建 `relocalization` 和 `mcu_bridge`，跳过 `rc_field_nav` 与 Gazebo 雷达插件；内存较小的机器可用 `JOBS=1 ./build.sh robot`。仅修改完整导航工作区后使用 `./build.sh nav`。单个第三方组件可用 `./third_party/build_deps.sh fast_livo` 等入口，详见 [维护文档](maintenance.md)。

建议使用 `~/SCU27RC_Nav` 这类 ASCII 路径。Humble 的接口生成在含中文的构建路径遇到列表解析问题；源码保留在中文父目录时，可将构建目录单独放到 ASCII 路径：

```bash
RC_DEPS_BUILD="$HOME/.cache/scu27rc/deps-build" \
RC_WS_BUILD="$HOME/.cache/scu27rc/nav-build" ./build.sh
```

后续构建使用相同设置，安装位置仍在当前仓库。默认构建采用普通安装；修改启动文件和参数后重新构建相应包，再重启。

源码复制到新路径、换电脑或换 CPU 架构后，在目标机器重新构建。部分依赖启用本机 CPU 优化，旧 `build/`、`install/` 还包含原机路径，不能作为跨机器安装包。

构建完成后确认解析到了正确包：

```bash
source env.sh
ros2 pkg prefix fast_livo
ros2 pkg prefix mcu_bridge
```

`fast_livo` 应来自本仓库 `third_party/install`，`mcu_bridge` 来自 `install`。完整仿真构建后再检查 `ros2 pkg prefix rc_field_nav`。默认 `ROS_DOMAIN_ID=42`、`GAZEBO_MASTER_URI=http://127.0.0.1:11355`；连接开发机与机器人时两侧 DDS 域应一致。

## 3. 先跑一次仿真

旧场地是默认入口，承接已有地图工作流。依次执行下面三步：

1. 终端 A：`./sim_map.sh`。无图形界面时使用 `./sim_map.sh gui:=false rviz:=false`。
2. 终端 B：`./sim_teleop.sh`，缓慢覆盖准备导航的区域。每个方向都观察点云与位姿是否一致。
3. 保持 A 运行，在终端 C 执行 `./sim_save_map.sh`，保存默认名 `prior`。确认点云、轨迹、栅格 YAML／图像已经保存，再用 Ctrl-C 停止 A、B。

随后运行 `./sim_nav.sh`。在 RViz 中观察地图与点云对齐，等待重定位被采纳，再在图内选择可通行的短距离目标。导航验证要同时查看目标结果、机器人实际运动和定位变化。

仿真地图默认保存在已安装包的 `share/rc_field_nav/maps`；`sim_nav.sh` 的选择规则与显式地图命令见 [地图与场地更新](maps.md)。保存地图时保持建图节点运行，因为累积点云在内存中。

### 使用国内 V3 征求意见稿场地

场地入口是 `field:=cn_v3_draft`，对应 2026-10-25 提供的国内 V3 征求意见稿资产。先阅读 [场地来源与验证范围](maps.md)。

```bash
./sim_map.sh field:=cn_v3_draft
# 另一终端遥控覆盖后，保持建图运行再存图
./sim_save_map.sh cn_v3_draft
```

停止建图和遥控后，明确选择刚保存的两份地图：

```bash
source env.sh
MAPS="$(ros2 pkg prefix rc_field_nav)/share/rc_field_nav/maps"
./sim_nav.sh field:=cn_v3_draft \
  prior_map:="${MAPS}/cn_v3_draft_map.pcd" map:="${MAPS}/cn_v3_draft_costmap.yaml"
```

建图与导航使用相同场地、出生点和机器人配置。旧场地的先验 PCD／二维图不能因文件名相同就用于 V3。

## 4. 真机先确认雷达与 LIO

Mid-360 的 IP 配置位于 [mid360_config.json](../robot/config/mid360_config.json)。真机驱动 launch 可根据雷达所在网段选择本机网卡 IP，必要时显式传入 `host_ip`、`lidar_ip`。将网卡配置到正确网段后再启动驱动。

```bash
source env.sh
ros2 launch "$PWD/robot/launch/real_livox_driver.launch.py" \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.126
```

以上地址来自仓库现有配置，换雷达或网卡时填写实物地址。另一个终端运行 `./check_lidar.sh`，确认 `/livox/lidar` 和 `/livox/imu` 持续发布、时间戳正常，再停止独立驱动并进入建图或导航入口。

当前共用 YAML 订阅原始 `/livox/imu`，`extrinsic_R` 为单位阵；不要根据旧部署记录额外启动 IMU 翻转。雷达安装变化时重新核对外参和静止重力方向，修改位置见 [源码索引](source_index.md)。

## 5. 选择真机运行方式

### 先验图导航

1. `./real_map.sh` 建图，固定雷达，慢速覆盖需要定位和导航的区域。
2. 保持建图运行，执行 `./real_save_map.sh --imu-height 0.30`。`0.30` 是例子，填写实测 IMU 离地高度，单位为米。
3. 停止建图，执行 `./real_reloc.sh rviz:=true`，观察 `/reloc_status`、点云与先验图是否对齐。
4. 完成定位验证后，停止定位入口，再执行 `./real_nav.sh` 启动带 Nav2 的链路。

`real_nav.sh` 输出 `/cmd_vel`。真机默认位置串口桥没有随该脚本启动；实际底盘需接通对应电控速度链路。若采用 `mcu_bridge` 的可选速度帧，先与电控核对协议并确保串口只由一个进程写入。

### MCU 位置闭环

```bash
MCU_DRY_RUN=1 ./start_robot.sh
```

此入口启动驱动、LIO 与 `mcu_bridge`，将当前位姿和 RViz 目标点编码为位置帧。`MCU_DRY_RUN=1` 时不打开串口，先观察 `/mcu_bridge/status` 与目标坐标；确认后停止入口，再用 `./start_robot.sh` 真发。观察机器人可运行 `./rviz_robot.sh`，Fixed Frame 为 `odom`。

串口设备、缩放、字节序、坐标符号在 [mcu_bridge.yaml](../src/mcu_bridge/config/mcu_bridge.yaml)。当前协议使用 `x` 向前、线上 `y` 向右、yaw 逆时针为正；转换应按配置和电控约定验证。

这两种运行方式根据电控需求选择。切换时先停止原有自动进程与串口写者，再启动目标入口。机器人可能存在已部署的 `robot-nav.service` 用户服务；若由它管理，使用 `systemctl --user stop robot-nav.service` 停止，避免服务自动重启后与手工联调重叠。服务是否安装以目标机状态为准。

## 6. 常见现象从哪里查

| 现象 | 优先查看 |
| --- | --- |
| 找不到 `fast_livo` 或工作区解析到旧路径 | 构建结果、`source env.sh`、`ros2 pkg prefix` |
| Livox 驱动构建失败 | 系统 Livox-SDK2 与 `third_party/README.md` 的前置 |
| TEASER++ configure 卡在下载 | 网络或 `third_party` 本地源码 |
| 有驱动进程却没有雷达话题 | DDS 域、网卡／雷达 IP、`./check_lidar.sh` 输出 |
| LIO 位姿快速发散 | 原始 IMU、时间戳、外参、实际加载的 fast_livo 版本 |
| RViz 有 map 帧但地图未对齐 | `/reloc_status`；bridge 在首次定位前可发布 identity TF |
| Nav2 发了速度但真机不动 | 底盘速度消费者、串口运行方式和电控状态 |
| 同一目标跑到错误场区 | PCD 与栅格配对、场地选择、出生点和坐标系 |
| 无 GPU／远程环境 Gazebo 无法启动 | [历史软件渲染配置](status.md#五验证环境无-gpu-的机器上跑仿真) |

运行记录写清使用的 Git 版本、机器、场地、地图、完整命令与观察结果，按 [维护文档](maintenance.md) 随改动更新。
