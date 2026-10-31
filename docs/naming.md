# 命名与目录迁移

本轮已把源码、真机资源、仿真场地和文档实际迁移到以下布局。ROS 包名、节点名、话题、串口协议及根目录一键命令保持兼容。

## 先按任务决定放哪里

| 内容 | 目录 | 例子 |
| --- | --- | --- |
| 本项目 ROS 实现 | `src/<package_name>/` | `src/relocalization/src/teaser_gicp_node.cpp` |
| 真机驱动装配与运行资源 | `robot/` | `robot/launch/real_relocalization.launch.py` |
| 真机／仿真共用配置 | `config/` | `config/livo_mid360.yaml` |
| 具体 ROS 包自己的参数 | 对应包的 `config/` | `src/mcu_bridge/config/mcu_bridge.yaml` |
| 场地、模型与生成源 | `simulation/worlds/` | `cn_v3_draft/source/` 与生成器 |
| 第三方实现 | `third_party/<upstream_name>/` | `third_party/fast_livo/` |
| 阅读、运行、维护说明 | `docs/` | `getting_started.md`、`source_index.md` |
| 比赛决策设计 | `decision/` | `01_framework.md`、`02_interfaces.md` |
| 原有联调记录 | `archive/legacy_runs/` | 旧脚本、服务文件与历史资料 |

`rc_field_nav` 保持仿真装配职责；真机入口独立位于 `robot/`。小电脑的 `robot` 构建模式只选择 `relocalization`、`mcu_bridge` 与当前真机依赖，不安装两个仿真包。

## 文件怎么命名

- 自有目录和普通文件使用 ASCII `snake_case`，名称表达任务或对象，例如 `source_index.md`、`mid360_config.json`、`sim_mapping.launch.py`。
- `real_*` 表示真机入口，`sim_*` 表示仿真入口；建图、重定位、导航分别命名，避免一个“start”文件包含多套不相干流程。
- ROS launch 保留 `.launch.py` 后缀，参数使用 `.yaml`，场地按实际格式使用 `.world`／`.sdf`。README、CMakeLists.txt、LICENSE 等工具和社区约定名称保留。
- 自有 ROS 包目录与包名一致。当前插件目录已改成 `src/ros2_livox_simulation`，包名原本就是 `ros2_livox_simulation`。
- 第三方源码的上游项目名、文件名、许可证和固定资源名保留；不能为统一大小写批量改动供应方代码。
- 地图名字表达场地或采集用途，例如 `cn_v3_draft_map.pcd` 与 `cn_v3_draft_costmap.yaml`；同次产物配套使用。版本与变更过程由 Git 记录。
- 生成资产修改生成源，再运行对应生成器。V3 的 `source/` 是维护输入，生成的 `models/`、`worlds/` 按 [地图说明](maps.md) 更新。

真机与仿真存在同名工具。`save_prior_map.sh` 的两份实现处理不同累积器与保存路径，分别保留；代码归并要先核对行为与调用方。

`src/relocalization/legacy/src/` 与 `legacy/launch/` 保存本包旧研究实现和旧启动文件。它们从当前源码／启动目录移出，不参与默认构建与安装；当前 `src/` 只保留 `teaser_gicp_node.cpp` 和 `transform_publisher.cpp`。

`third_party/COLCON_IGNORE` 将第三方目录排除在根工作区的自动发现之外；`build.sh` 再显式调用第三方构建入口。它沿用构建工具的目录规则。

## 旧新路径对照

表中路径均相对仓库根目录；多步迁移按顺序发生，例如先移动 `Docs/`，再把其中的中文文件改成 ASCII 名称。

| 原路径 | 当前路径 |
| --- | --- |
| `Nav/ros2_ws/src` | `src` |
| `Deps` | `third_party` |
| `Docs` | `docs` |
| `Decide` | `decision` |
| `Nav/real` | `robot` |
| `Nav/config` | `config` |
| `Nav/sim` | `simulation` |
| `Nav/README.md` | `docs/navigation.md` |
| `logs` | `archive/legacy_runs` |
| `src/livox_laser_simulation_ros2` | `src/ros2_livox_simulation` |
| `docs/上手与部署.md` | `docs/getting_started.md` |
| `docs/源码与配置索引.md` | `docs/source_index.md` |
| `docs/仓库维护与更新.md` | `docs/maintenance.md` |
| `docs/地图与场地更新.md` | `docs/maps.md` |
| `docs/比赛项目参考.md` | `docs/references.md` |
| `docs/项目结构.md` | `docs/architecture.md` |
| `docs/一键启动脚本目录.md` | `docs/scripts.md` |
| `docs/项目当前情况.md` | `docs/status.md` |
| `decision/01-框架设计.md` | `decision/01_framework.md` |
| `decision/02-接口契约.md` | `decision/02_interfaces.md` |
| `decision/03-策略落地映射.md` | `decision/03_strategy_mapping.md` |
| `decision/04-可行性分析.md` | `decision/04_feasibility.md` |
| `robot/config/MID360_config.json` | `robot/config/mid360_config.json` |
| `robot/launch/real_reloc.launch.py` | `robot/launch/real_relocalization.launch.py` |
| `robot/launch/real_localization.launch.py` | `robot/launch/real_voxel_localization.launch.py` |
| `src/rc_field_nav/launch/lio_map.launch.py` | `src/rc_field_nav/launch/sim_mapping.launch.py` |
| `src/rc_field_nav/launch/nav_prior_map.launch.py` | `src/rc_field_nav/launch/sim_navigation.launch.py` |

根 `build/`、`install/`、`log/` 替代原 `Nav/ros2_ws/` 下的构建产物；第三方安装前缀为 `third_party/install/`。旧构建缓存与二进制不随源码迁移到新机器，应在目标位置重新编译。

已有机器通过 Git 更新后，旧目录里的未跟踪产物可能仍留在磁盘上。`.gitignore` 保留对这些旧构建目录的排除，当前环境与入口加载新安装前缀。

重定位研究代码另做以下归档：

| 原位置 | 当前位置 |
| --- | --- |
| `src/relocalization/src/` 中的 `icp_node.cpp`、`sac_ia_gicp.cpp`、`small_gicp_node.cpp`、`teaser_node.cpp` | `src/relocalization/legacy/src/` |
| `src/relocalization/launch/` 中的 `icp.launch.py`、`sac_ia_gicp.launch.py`、`small_gicp.launch.py`、`teaser.launch.py` | `src/relocalization/legacy/launch/` |

## 后续移动文件时一起更新

1. 启动脚本和 launch 的根路径发现、资源路径与参数默认值。
2. CMake／Python 包安装规则、构建脚本和环境加载位置。
3. ROS 包引用、场地 URI 与地图选择逻辑；包名和节点接口按现有约定维护。
4. 对应 Markdown 的链接、目录树和可执行示例。
5. 从新位置构建并运行受影响流程，确认读取的是新目录中的源码与配置。

历史测量、旧机器绝对路径和当时命令保留在记录中，便于追溯；历史 Markdown 链接修到当前资料。日常使用查 [部署说明](getting_started.md) 和 [脚本目录](scripts.md)，不要从归档脚本推断默认入口。

这套组织借鉴 Nav2 和 pb2025 的职责分离、RoboRTS 的分包方式，以及 FAST_LIO 的包内源码与配置布局；关注点是本仓库的运行边界，来源见 [工程组织参考](references.md)。
