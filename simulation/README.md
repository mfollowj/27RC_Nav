# simulation —— 仿真场地与资源

- 新人从 [部署说明](../docs/getting_started.md) 进入；地图维护看 [地图与场地更新](../docs/maps.md)。
- 整车导航使用 Gazebo Classic 11；现用 Docker `rc27_nav` 的 Fortress 6.18.0 可单独展示 V3 场地，两个引擎的整车插件分别处理。
- V3 草案通过 `./sim_map.sh field:=cn_v3_draft` 选择；默认 `legacy` 衔接旧先验图。换场地后重新建图。
- `worlds/`：场地 world、BR 机器人模型、Mid-360 模型。
- 启动：仓库根 `./sim_map.sh`、`./sim_nav.sh`、`./sim_teleop.sh`、`./sim_save_map.sh`。
- LIO：**FAST-LIVO2（无视觉）**，与真机共用 `config/livo_mid360.yaml`；
  抽稀节点也是真机那个 `livox_preprocess.py`（同一话题 `/livox/lidar_filtered`）。
- 重定位：`sim_navigation.launch.py` 走真机那条
  `teaser_gicp_node -> reloc_bridge -> /relocalization_result + map->odom`，
  LIO 以 `locate_in_prior_map` 跟踪先验图；旧的 4-DOF `prior_map_localizer`
  仍保留，用 `reloc_impl:=voxel` 切换。
- 雷达插件源码：`src/ros2_livox_simulation/`。
- 无 GPU 的机器跑仿真需要 Mesa 软件渲染，见 `docs/status.md` 第五节。
