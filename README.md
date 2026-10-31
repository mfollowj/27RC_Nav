# SCU27RC_Nav

四川大学穿山甲战队的 2027 ROBOCON 导航仓库，包含真机与仿真导航、雷达与串口接入，以及比赛决策层的设计文档。

**第一次接手，从 [新人阅读入口](docs/README.md) 开始；准备运行，按 [上手与部署](docs/getting_started.md) 操作；准备修改，查 [源码与配置索引](docs/source_index.md)。**

## 仓库在做什么

- 导航主链：Mid-360 点云与 IMU → 抽稀 → FAST-LIVO2（无视觉模式）→ TEASER++／VGICP 重定位 → Nav2。
- 仿真侧：Gazebo Classic 场地、四全向轮机器人和 Mid-360 插件；与真机共用 LIO 参数及重定位实现。
- 小机器人串口链：LIO 当前位姿与 RViz 目标点 → `mcu_bridge` → MCU。该入口由 MCU 执行位置闭环；Nav2 导航另由 `real_nav.sh` 启动。
- `decision/`：现有 5 份设计文档，说明比赛阶段、任务、导航与机构接口；可运行的决策节点仍待实现。

已有的 2026-10-30 仿真建图、存图、重定位和目标到达记录保存在 [项目当前情况](docs/status.md)。真机部署状态、当前改动与待验证项在同一文档列出。

## 目录怎么分

```text
src/                   4 个本项目 ROS 包
robot/                 真机运行入口、配置、脚本与地图
config/                真机与仿真共用的 LIO 参数
simulation/            场地、模型与生成源
third_party/           第三方算法与驱动源码
decision/              比赛决策设计
docs/                  阅读、运行、修改与维护说明
archive/legacy_runs/    历史联调资料
```

根目录是 colcon 工作区，`src/` 下有 `rc_field_nav`、`relocalization`、`ros2_livox_simulation` 和 `mcu_bridge`。本轮已实际迁移文件，保留包名与运行接口；目录与文件的旧新对照见 [命名与迁移](docs/naming.md)。

## 第一次运行

目标系统为 **Ubuntu 22.04、ROS 2 Humble**；整车仿真使用 **Gazebo Classic 11**，真机模式不构建该仿真插件。还需 Nav2、PCL／Eigen／OpenCV／Sophus 和系统安装的 Livox-SDK2。第三方源码随仓库提供，构建产物安装到 `third_party/install`；系统依赖仍须单独安装。TEASER++ 首次构建会下载依赖，也可按 [Deps 说明](third_party/README.md) 提前准备本地源码。

在目标机器的 `~/SCU27RC_Nav` 等 ASCII 路径获取源码并构建；跨机器、路径或 CPU 架构时重新编译，勿复制旧 `build/`、`install/` 作为部署方法。

```bash
cd ~
git clone https://github.com/SCU-Pangolin/SCU27RC_Nav.git
cd SCU27RC_Nav
./build.sh
source env.sh
```

默认仿真保留旧场地，用于衔接原有先验图流程。先建图，再在另一终端遥控与存图；存图完成后停止建图和遥控，再启动导航：

```bash
./sim_map.sh                         # 终端 A
./sim_teleop.sh                      # 终端 B，慢速走过待导航区域
./sim_save_map.sh                    # 终端 C，保持 A 运行，保存默认 prior 地图
# 停止 A、B 后，在仓库根目录运行
./sim_nav.sh
```

**国内 V3 征求意见稿场地**通过 `./sim_map.sh field:=cn_v3_draft` 显式选择。切换场地后重新制作匹配的点云与栅格地图，按 [地图与场地更新](docs/maps.md) 启动；V3 场地名称与旧先验图是两类资源。

## 常用入口

| 任务 | 命令 | 说明 |
| --- | --- | --- |
| 机器人小电脑构建 | `JOBS=1 ./build.sh robot` | 构建依赖、重定位和串口包，跳过两个仿真包 |
| 全量构建／只构建项目包 | `./build.sh`／`./build.sh nav` | 系统依赖准备好后执行 |
| 仿真建图／导航 | `./sim_map.sh`／`./sim_nav.sh` | 导航需匹配的先验图 |
| 真机建图／存图 | `./real_map.sh`／`./real_save_map.sh --imu-height 0.30` | 高度按实车测量，单位为米 |
| 真机仅定位／Nav2 导航 | `./real_reloc.sh`／`./real_nav.sh` | Nav2 输出速度的执行接入见部署文档 |
| 小机器人位置串口联调 | `MCU_DRY_RUN=1 ./start_robot.sh` | 驱动、LIO、位置串口桥；首次先检查输出 |
| 本机查看机器人 | `./rviz_robot.sh` | `odom` 为 Fixed Frame |

参数与停止方式见 [一键启动脚本目录](docs/scripts.md)。手工运行 `ros2` 命令前执行 `source env.sh`，默认 DDS 域为 42、Gazebo 端口为 11355。

## 按任务找文档

- [新人阅读入口](docs/README.md)：先看哪里、各目录与模块如何连接。
- [上手与部署](docs/getting_started.md)：环境、构建、仿真、真机和常见排障。
- [源码与配置索引](docs/source_index.md)：改机器人、地图、导航、驱动或串口时找哪个文件。
- [命名与迁移](docs/naming.md)：目录职责、snake_case 命名和旧新路径对照。
- [仓库维护与更新](docs/maintenance.md)：分支、提交、PR、验证、依赖与文档同步。
- [地图与场地更新](docs/maps.md)：V3 来源、场地生成与配套地图。
- [项目当前情况](docs/status.md)：已实现能力、历史实测与待完成事项。
- [比赛项目参考](docs/references.md)：高 star 工程组织参考与 10 个近期比赛场景样本。
- [Decide 索引](decision/README.md)：开始实现比赛任务前阅读。
