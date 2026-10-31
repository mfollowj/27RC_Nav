# Deps —— 非官方依赖（随仓库打包）

这里放的是**不能直接 `apt install ros-humble-*` 拿到**的依赖源码。目标：换一台机器时，
只要有 ROS 2 Humble + Gazebo + PCL，跑 `./Deps/build_deps.sh` 就能把整套链路编出来。

> **状态（2026-10-30）：全部编译通过，`Deps/install` 是唯一依赖来源。**
> `env.sh` 里已经删掉了所有外部工作区兜底（旧 `SCURC_Nav_Sim`、`getting_relocal`），
> 真机与仿真都只 source 本目录 + `Nav/ros2_ws`。

| 目录 | 包名 | 用途 | 来源 |
| --- | --- | --- | --- |
| `fast_livo/` | `fast_livo`（FAST-LIVO2） | 真机/仿真的 LIO（**无视觉**模式），**两边共用** | SCURC_Nav_Sim 中的本地分支，**含本项目补丁**（`need_map_reinit`、odom 归零、重力旋转、退出汇总） |
| `rpg_vikit/` | `vikit_common` / `vikit_ros` / `vikit_py` | FAST-LIVO2 依赖 | rpg_vikit |
| `livox_ros_driver2/` | `livox_ros_driver2` | Mid-360 驱动（CustomMsg） | Livox 官方 ROS2 驱动 |
| `fast_lio/` | `fast_lio` | **已不被仿真使用**（仿真 2026-10-30 切到 fast_livo），保留用于对照/回退 | SCURC_Nav_Sim 分支 |
| `small_gicp/` | `small_gicp`（CMake） | 6-DOF 重定位的精配准 | small_gicp |
| `TEASER-plusplus/` | `teaserpp`（CMake） | 6-DOF 全局重定位 | TEASER++ |

## 系统级依赖（不在本目录，需要宿主提供）

| 依赖 | 安装方式 |
| --- | --- |
| ROS 2 Humble、Gazebo Classic 11 | apt（`ros-humble-desktop`、`ros-humble-gazebo-*`） |
| Nav2 | apt（`ros-humble-navigation2`、`ros-humble-nav2-bringup`） |
| PCL / Eigen / OpenMP / OpenCV / Sophus | apt（`libpcl-dev`、`libeigen3-dev`、`libopencv-dev`、`ros-humble-sophus`） |
| **Livox-SDK2** | 源码编译安装到 `/usr/local`（livox 驱动的硬依赖） |

## 编译

```bash
./Deps/build_deps.sh              # 全量（small_gicp -> teaser -> vikit -> livox -> fast_livo -> fast_lio）
./Deps/build_deps.sh fast_livo    # 只编某一个：small_gicp / teaser / vikit / livox / fast_livo / fast_lio
JOBS=8 ./Deps/build_deps.sh       # 并行度（默认 4）
```

产物统一装到 `Deps/install/`：

```
Deps/install/
├── setup.bash                 # colcon merge-install 的环境（env.sh 自动 source）
├── share/                     # ament 包：fast_livo / fast_lio / livox_ros_driver2 / vikit_*
├── lib/                       # 可执行文件 + libteaser_*.so + libsmall_gicp.so
├── include/
└── lib/cmake/{teaserpp,small_gicp}   # 普通 CMake 包的 find_package 入口
```

- `env.sh` 会自动 source `Deps/install/setup.bash`，并把 `Deps/install/lib` 加进
  `LD_LIBRARY_PATH`（TEASER++/small_gicp 不是 ament 包，ld 不会自己找到）。
- `Nav/ros2_ws` 的 `relocalization` 包编译时靠 `Deps/install/lib/cmake/teaserpp` 找到
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
离线时把这三个目录放到 `TEASER-plusplus/third_party/<name>`，`build_deps.sh` 会自动改用本地源
（`-DFETCHCONTENT_SOURCE_DIR_<NAME>`）。
