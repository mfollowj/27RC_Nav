# 新人阅读入口

这套仓库解决三件事：让机器人知道自己在哪、导航到目标、把必要数据交给电控。比赛决策的阶段和任务接口已经有设计文档，代码由后续开发接入。

## 第一天怎么读

1. 阅读 [根 README](../README.md) 和 [项目当前情况](status.md)，了解当前实现、运行记录与待办。
2. 按 [上手与部署](getting_started.md) 准备环境，运行一次仿真建图、存图与导航。
3. 对照 [项目结构](architecture.md) 找到目录，再沿 [源码与配置索引](source_index.md) 追踪正在运行的链路。
4. 第一次修改前阅读 [仓库维护与更新](maintenance.md)，把源码、参数、文档与验证结果放在同一次改动里。

无需先通读 `third_party/` 的算法源码。先弄清启动入口加载了哪个包、参数和地图，再阅读对应实现。

## 按工作找入口

| 你要做什么 | 先读 | 接着看 |
| --- | --- | --- |
| 跑仿真或接手机器人 | [上手与部署](getting_started.md) | [脚本目录](scripts.md) |
| 了解 src 包怎么分类 | [项目结构](architecture.md) | [源码与配置索引](source_index.md)、[命名与迁移](naming.md) |
| 改 V3 场地或重做先验图 | [地图与场地更新](maps.md) | 建图、存图与导航脚本 |
| 调定位、底盘、Nav2 | [源码与配置索引](source_index.md) | 相应 launch、YAML 与实现 |
| 接雷达、串口或部署 | [上手与部署](getting_started.md) | [mcu_bridge 说明](../src/mcu_bridge/README.md) |
| 更新依赖或提交代码 | [仓库维护与更新](maintenance.md) | [第三方依赖说明](../third_party/README.md) |
| 开始做比赛任务 | [Decide 索引](../decision/README.md) | 其中的框架、接口与可行性分析 |
| 参考其他比赛队伍 | [比赛项目参考](references.md) | 对应原仓库的实际源码与参数 |

## 一条导航命令背后发生什么

```text
根目录 sim_*.sh / real_*.sh → env.sh → 对应 launch
雷达 + IMU → 抽稀 → FAST-LIVO2 → 局部里程计与点云
先验 PCD + 当前点云 → TEASER++/VGICP → reloc_bridge → map→odom
栅格图 + TF + 感知数据 → Nav2 → /cmd_vel → 底盘执行
```

仿真中 `/cmd_vel` 由 `omni_drive.py` 消费，再转换为轮速。真机默认 `real_nav.sh` 启动导航与定位，速度下发由具体电控接入决定。`start_robot.sh` 是另一条“当前位姿／目标点交给 MCU”的运行方式；默认位置协议与 Nav2 速度协议的区别见 [串口包说明](../src/mcu_bridge/README.md)。

## src 与其他目录的关系

根目录是 colcon 工作区，4 个项目包平铺在 `src/` 下，按职责分工：

| 包 | 负责什么 | 阅读起点 |
| --- | --- | --- |
| `rc_field_nav` | 仿真装配、机器人模型、Nav2 配置、建图与存图工具 | [launch](../src/rc_field_nav/launch) |
| `relocalization` | TEASER++／VGICP 全局重定位 | [CMakeLists.txt](../src/relocalization/CMakeLists.txt)、[主节点](../src/relocalization/src/teaser_gicp_node.cpp) |
| `ros2_livox_simulation` | Gazebo 中生成 Mid-360 点云 | [插件目录](../src/ros2_livox_simulation) |
| `mcu_bridge` | 当前位姿与目标点的串口帧，可选速度帧 | [README](../src/mcu_bridge/README.md) |

`robot/` 保存真机 launch 与辅助脚本，`config/` 保存共用 LIO 参数，`simulation/worlds/` 保存场地资产。`third_party/` 保存第三方源码，`decision/` 保存比赛决策设计。目录已按这些职责实际迁移，包名和运行接口保留，后续按 [命名规则](naming.md) 放置新文件。

## 阅读状态与历史记录

[项目当前情况](status.md) 区分当前代码、原有实测和后续任务。历史记录中的机器路径、服务部署和先验图数值描述当时的机器；新机器按部署文档重新构建，运行结果记录实际分支、场地、命令与现象。

## Markdown 资料在哪里

根 README 和 `docs/` 负责全仓阅读与运行；`decision/` 保存比赛任务设计；各 ROS 包 README 负责本包接口；`third_party/` 内的 README 主要是第三方原始说明；`archive/legacy_runs/` 内的 Markdown 保存联调记录。在 Git 检出中运行 `git ls-files '*.md'` 可列出当前版本全部受跟踪的 Markdown。
