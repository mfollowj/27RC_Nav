# Nav/sim —— 仿真侧

- `worlds/`：场地 world、BR 机器人模型、Mid-360 模型。
- 启动：仓库根 `./sim_map.sh`、`./sim_nav.sh`、`./sim_teleop.sh`、`./sim_save_map.sh`。
- LIO：**FAST-LIVO2（无视觉）**，与真机共用 `Nav/config/livo_mid360.yaml`；
  抽稀节点也是真机那个 `livox_preprocess.py`（同一话题 `/livox/lidar_filtered`）。
- 重定位：`nav_prior_map.launch.py` 走真机那条
  `teaser_gicp_node -> reloc_bridge -> /relocalization_result + map->odom`，
  LIO 以 `locate_in_prior_map` 跟踪先验图；旧的 4-DOF `prior_map_localizer`
  仍保留，用 `reloc_impl:=voxel` 切换。
- 雷达插件源码：`Nav/ros2_ws/src/livox_laser_simulation_ros2/`。
- 无 GPU 的机器跑仿真需要 Mesa 软件渲染，见 `Docs/项目当前情况.md` 第五节。
