# 项目结构

更新时间：2026-10-31。本轮已实际迁移源码、资源与文档；文件命名和旧新路径对照见 [命名与迁移](naming.md)。

## 按职责找目录

```text
SCU27RC_Nav/
├── src/                    4 个本项目 ROS 包
├── robot/                  真机 launch、脚本、配置、地图与 RViz
├── config/                 真机与仿真共用的 LIO 参数
├── simulation/worlds/      仿真场地、模型与生成源
├── third_party/            随仓库维护的第三方实现与构建脚本
├── decision/               比赛决策设计，当前 5 份 Markdown
├── docs/                   阅读、部署、修改与维护文档
├── archive/legacy_runs/    历史联调脚本、服务文件与记录
├── build.sh / env.sh        构建与环境入口
└── sim_*.sh / real_*.sh     保留的一键运行命令
```

根目录是本项目的 colcon 工作区。项目构建产物位于根 `build/`、`install/`、`log/`；第三方产物位于 `third_party/install/`，不作为跨机器安装包复制。

## src：四个 ROS 包

| 包目录 | 职责 | 构建范围 |
| --- | --- | --- |
| [rc_field_nav](../src/rc_field_nav/) | 仿真装配、机器人模型、Nav2 参数、底盘与地图工具 | 完整仿真 |
| [relocalization](../src/relocalization/) | TEASER++／VGICP 全局重定位 | 真机与仿真 |
| [ros2_livox_simulation](../src/ros2_livox_simulation/) | Gazebo Classic Mid-360 插件 | 完整仿真 |
| [mcu_bridge](../src/mcu_bridge/) | 当前位姿、目标点与可选速度串口协议 | 真机 |

包名、节点名、话题和协议沿用原实现。迁移后插件目录与包名均为 `ros2_livox_simulation`。重定位包的旧研究实现和无效旧 launch 分别归入 `legacy/src`、`legacy/launch`，当前构建只保留两个已有目标。

`./build.sh` 构建依赖与全部项目包；`JOBS=1 ./build.sh robot` 构建真机依赖及 `relocalization`、`mcu_bridge`，跳过两个仿真包和对照用 `fast_lio`。这样小电脑无需为位置串口链安装 Classic 仿真插件。

## 真机、共用配置与场地资源

| 目录 | 内容 | 入口 |
| --- | --- | --- |
| [robot/launch](../robot/launch/) | 驱动、建图、重定位、Nav2 装配 | `real_map.sh`、`real_reloc.sh`、`real_nav.sh` |
| [robot/scripts](../robot/scripts/) | 点云处理、定位桥、轨迹与地图检查 | 由真机 launch／存图脚本调用 |
| [robot/start_robot.sh](../robot/start_robot.sh) | 驱动、LIO 与 MCU 位置闭环接入 | 根 `start_robot.sh` 转调 |
| [config](../config/) | `livo_mid360.yaml`、`camera_pinhole.yaml` | real／sim launch 共用，平台差异在 launch 覆盖 |
| [simulation/worlds](../simulation/worlds/) | 旧场地与国内 V3 征求意见稿场地 | `field:=legacy`／`field:=cn_v3_draft` |

真机先验图位于 `robot/maps/`；仿真地图默认保存在已安装包的 `share/rc_field_nav/maps/`。场地模型、先验 PCD 与二维图的对应关系见 [地图与场地更新](maps.md)。

## 两条真机执行流程

```text
real_nav.sh     → 雷达/LIO/重定位/Nav2 → /cmd_vel → 接入的底盘速度链
start_robot.sh  → 雷达/LIO → 当前位姿 + RViz 目标 → mcu_bridge → MCU
```

仿真导航由 `omni_drive.py` 消费速度并驱动车轮；真机按电控接口选择速度闭环或位置闭环。操作步骤见 [上手与部署](getting_started.md)，文件级修改入口见 [源码与配置索引](source_index.md)。

## 第三方与决策设计

[third_party](../third_party/README.md) 保存 FAST-LIVO2、vikit、Livox 驱动、small_gicp、TEASER++ 等源码和本地修改；ROS／Nav2、系统库与 Livox-SDK2 由目标系统提供。上游项目名与许可证保留。

`third_party/COLCON_IGNORE` 使用 colcon 的标准目录排除方式，让根工作区只发现 4 个自有包。根构建入口另行显式选择第三方源码，按“第三方依赖 → 自有包”分阶段编译。

[decision](../decision/README.md) 当前只有设计文档。设计中的决策包、配置、任务 launch 与机构接口是后续实现目标；未来 ROS 实现仍接入根 `src/`，按包表达职责。

`env.sh` 加载 Humble、`third_party/install` 与根 `install`，默认 DDS 域 42、Gazebo 端口 11355。历史软链与旧部署路径保存在 [运行记录](status.md) 中。
