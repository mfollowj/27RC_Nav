# relocalization：全局重定位

本包把实时 LiDAR 点云与先验 PCD 配准，当前默认流程由 TEASER++ 做全局初始化，再由 small_gicp 的 VGICP／GICP 跟踪与精配准。完整导航由仓库根入口装配，包内研究代码单独保留。

## 当前构建内容

| 目标 | 源文件 | 用途 |
| --- | --- | --- |
| `teaser_gicp_node` | [src/teaser_gicp_node.cpp](src/teaser_gicp_node.cpp) | 当前全局重定位节点 |
| `transform_publisher` | [src/transform_publisher.cpp](src/transform_publisher.cpp) | 兼容旧流程的 TF 发布器；当前主链使用定位桥 |

[CMakeLists.txt](CMakeLists.txt) 只构建这两个目标并安装 RViz 资源。`legacy/src/` 保存旧 ICP、SAC-IA、small_gicp、TEASER 实验实现，`legacy/launch/` 保存对应旧启动文件；它们不参与默认构建或安装，部分旧启动文件仍含原作者路径。研究时先阅读实现和依赖，当前运行使用下方入口。

## 从仓库根目录构建

系统 ROS 2 Humble、PCL、Eigen、OpenMP 和 Livox-SDK2 准备完成后：

```bash
JOBS=1 ./build.sh robot
source env.sh
```

已构建依赖后，仅重建本包使用 `./build.sh nav --packages-select relocalization`。

TEASER++ 和 small_gicp 由 [第三方构建入口](../../third_party/README.md) 编译为 CMake 库，安装到 `third_party/install`。TEASER++ 的 FPFH 支持已由该入口启用；需要单独重建时从根目录执行 `./third_party/build_deps.sh teaser`，再重建本包。

当前 CMake 启用 Livox 支持，构建依赖包含 `livox_ros_driver2`。节点运行时按 `pcl_type` 选择 Livox CustomMsg 或标准 PointCloud2 输入。

## 当前运行入口

```bash
./real_reloc.sh rviz:=true
```

此入口启动真机驱动、LIO、重定位节点与 `robot/scripts/reloc_bridge.py`，默认不开 Nav2。默认先验为 `robot/maps/prior_map.pcd`，可通过 `map_path:=` 指定实际 PCD。

- 真机带 Nav2：`./real_nav.sh`。
- 仿真：`./sim_nav.sh`，由 `src/rc_field_nav/launch/sim_navigation.launch.py` 装配同一重定位实现。
- 入口参数：查看 [robot/launch/real_relocalization.launch.py](../../robot/launch/real_relocalization.launch.py)；地图配对和坐标要求见 [地图说明](../../docs/maps.md)。

## 数据与参数

节点在 Livox 模式订阅 `/livox/lidar`，标准模式订阅 `/pointcloud2`，并可接收 `initialpose`。它发布 `icp_result`、`prior_map` 和 `transformed_cloud`。

在当前导航链中，`icp_result` 是传感器配准结果，定位桥负责换算为车体位姿、采纳稳定结果并更新 `map→odom`；仅启动配准节点不等于完整 LIO／TF／Nav2 链已经就绪。

| 目的 | 参数 |
| --- | --- |
| 先验与坐标系 | `map_path`、`map_frame_id`、`pcl_type` |
| 全局搜索降采样 | `map_voxel_leaf_size`、`cloud_voxel_leaf_size` |
| 全局特征与求解 | `fpfh_normal_radius`、`fpfh_feature_radius`、`noise_bound` |
| 全局初始化采纳 | `teaser_inlier_threshold`、`teaser_success_count` |
| 局部精配准 | `gicp_map_voxel_leaf_size`、`gicp_cloud_voxel_leaf_size`、`registration_type` |
| 局部结果与输出 | `fitness_score_thre`、`converged_count_thre`、`publish_after_converged` |

根入口的 `map_voxel`、`teaser_inlier` 等是装配参数，由 launch 转给节点；真机与仿真默认值不同。调参时先确认地图覆盖、点云与先验对齐，再记录内点、收敛和定位表现。

RViz 使用 `map` 为 Fixed Frame，观察 `prior_map` 和 `transformed_cloud`；完整链路同时查看 `/reloc_status` 与 TF。源码修改位置及其他发布者见 [源码索引](../../docs/source_index.md)。

## 来源与致谢

保留原实现的来源记录：

- 基础框架、ICP 与 SAC-IA 研究实现：[PolarisXQ/Fast-LIO2-Localization](https://github.com/PolarisXQ/Fast-LIO2-Localization)。
- 全局鲁棒配准：[MIT-SPARK/TEASER-plusplus](https://github.com/MIT-SPARK/TEASER-plusplus)。
- 局部精配准：[koide3/small_gicp](https://github.com/koide3/small_gicp)。

第三方升级和本地修改按 [维护说明](../../docs/maintenance.md) 记录，上游许可证与引用信息随对应源码保留。
