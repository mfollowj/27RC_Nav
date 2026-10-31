# third_party —— 随仓库维护的第三方源码

这里保存本项目维护的第三方算法与驱动源码。系统依赖准备好后，使用 `./third_party/build_deps.sh` 将产物安装到仓库内；全仓构建从根目录 `./build.sh` 进入。

> **历史验证（2026-10-30）：全部编译通过，`Deps/install` 是该次运行的依赖来源。**
> `env.sh` 里已经删掉了所有外部工作区兜底（旧 `SCURC_Nav_Sim`、`getting_relocal`），
> 真机与仿真都只 source 本目录 + `Nav/ros2_ws`。

上方路径是原记录。本轮源码已迁到 `third_party/`，项目包位于根 `src/`；当前环境加载 `third_party/install` 与根 `install`，目录规则见 [命名与迁移](../docs/naming.md)。

本目录有 `COLCON_IGNORE`，因此根 `colcon` 默认只发现 4 个自有包。`build.sh` 会先进入本目录的显式依赖构建，再编译根 `src/`，第三方 ROS 包不与自有包混在同一发现范围中。

| 目录 | 包名 | 用途 | 来源 |
| --- | --- | --- | --- |
| `fast_livo/` | `fast_livo`（FAST-LIVO2） | 真机/仿真的 LIO（**无视觉**模式），**两边共用** | SCURC_Nav_Sim 中的本地分支，**含本项目补丁**（`need_map_reinit`、odom 归零、重力旋转、退出汇总） |
| `rpg_vikit/` | `vikit_common` / `vikit_ros` / `vikit_py` | FAST-LIVO2 依赖 | rpg_vikit |
| `livox_ros_driver2/` | `livox_ros_driver2` | Mid-360 驱动（CustomMsg） | Livox 官方 ROS2 驱动 |
| `fast_lio/` | `fast_lio` | **已不被仿真使用**（仿真 2026-10-30 切到 fast_livo），保留用于对照/回退 | SCURC_Nav_Sim 分支 |
| `small_gicp/` | `small_gicp`（CMake） | 6-DOF 重定位的精配准 | small_gicp |
| `TEASER-plusplus/` | `teaserpp`（CMake） | 6-DOF 全局重定位 | TEASER++ |

## 系统级依赖（不在本目录，需要宿主提供）

| 使用范围 | 依赖 | 安装方式 |
| --- | --- | --- |
| 真机与仿真 | ROS 2 Humble、colcon 与编译工具 | Ubuntu 22.04 的 Humble 环境；有 RViz 需求可用 `ros-humble-desktop` |
| 真机与仿真 | Nav2 | apt（`ros-humble-navigation2`、`ros-humble-nav2-bringup`） |
| 真机与仿真 | PCL / Eigen / OpenMP / OpenCV / Sophus / fmt | apt（`libpcl-dev`、`libeigen3-dev`、`libomp-dev`、`libopencv-dev`、`ros-humble-sophus`、`libfmt-dev`） |
| 真机与仿真 | **Livox-SDK2** | 按下方官方入口源码编译安装到 `/usr/local`；Livox 驱动需要它 |
| Classic 整车仿真附加 | Gazebo Classic 11、Gazebo ROS、gazebo_ros2_control、轮速控制器 | 对应 Humble 的 Gazebo ROS 与控制器系统包，详见 [部署清单](../docs/getting_started.md) |

Python 工具还使用 `python3-numpy`、`python3-yaml`、`python3-pil`、`python3-serial`。它们与 ROS／SDK 都属于目标系统安装内容。当前 Docker `rc27_nav` 的 Gazebo 是 Fortress 6.18.0；Classic 仿真插件需要匹配的 Classic 11 开发与 ROS 包。

Livox-SDK2 的源码与系统安装步骤见 [Livox 官方安装文档](https://github.com/Livox-SDK/Livox-SDK2#2-installation)。在仓库外的工作目录准备 SDK，安装后再运行本仓库构建：

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cmake -S Livox-SDK2 -B Livox-SDK2/build
cmake --build Livox-SDK2/build -j1
sudo cmake --install Livox-SDK2/build
sudo ldconfig
```

官方默认将头文件和库装到 `/usr/local/include`、`/usr/local/lib`。已有可用 SDK 的机器沿用现有安装；队内更新 SDK 时记录所用版本及驱动联调结果。

## 编译

```bash
./third_party/build_deps.sh              # 全量（small_gicp -> teaser -> vikit -> livox -> fast_livo -> fast_lio）
./third_party/build_deps.sh robot        # 当前真机链路依赖，跳过对照用 fast_lio
./third_party/build_deps.sh fast_livo    # 只编某一个：small_gicp / teaser / vikit / livox / fast_livo / fast_lio
JOBS=1 ./third_party/build_deps.sh       # 内存较少时串行编译（默认 2）
```

产物统一装到 `third_party/install/`：

```
third_party/install/
├── local_setup.bash           # env.sh 加载本前缀，避免引入构建机的旧工作区
├── share/                     # ament 包：fast_livo / fast_lio / livox_ros_driver2 / vikit_*
├── lib/                       # 可执行文件 + libteaser_*.so + libsmall_gicp.so
├── include/
└── lib/cmake/{teaserpp,small_gicp}   # 普通 CMake 包的 find_package 入口
```

- `env.sh` 会自动 source `third_party/install/local_setup.bash`，并把 `third_party/install/lib` 加进
  `LD_LIBRARY_PATH`（TEASER++/small_gicp 不是 ament 包，ld 不会自己找到）。
- Livox ROS 2 的 `package.xml` 由 `package_ROS2.xml` 生成；依赖构建入口会在全量或驱动构建时完成这一步。后续组件构建会加载前面已经安装的依赖。
- 根 `src/relocalization` 编译时靠 `third_party/install/lib/cmake/teaserpp` 找到
  TEASER++，**不再需要** `-DCMAKE_PREFIX_PATH=<外部工作区>`。

## 实测记录（2026-10-30，本机 24 核）

| 包 | 结果 | 备注 |
| --- | --- | --- |
| small_gicp | ✅ | 普通 CMake，几十秒 |
| TEASER++ | ✅ | 首次 configure 会用 FetchContent 从 GitHub 拉 `pmc / tinyply / spectra`（**需要联网**），编译约 2 min |
| vikit_common / vikit_ros / vikit_py | ✅ | ~22 s |
| livox_ros_driver2 | ✅ | ~15 s；需要系统 Livox-SDK2 |
| fast_livo | ✅ | ~65 s |
| fast_lio | ✅ | ~65 s（当前不被仿真使用） |

## 离线机器

TEASER++ 首次 configure 默认会从 GitHub 拉 `pmc / tinyply / spectra`。
从仓库根目录准备离线源码时，将这三个目录放到 `third_party/TEASER-plusplus/third_party/<name>`，构建入口会自动改用本地源
（`-DFETCHCONTENT_SOURCE_DIR_<NAME>`）。
