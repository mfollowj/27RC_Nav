# simulation —— 仿真场地与资源

- 新人从 [部署说明](../docs/getting_started.md) 进入；地图维护看 [地图与场地更新](../docs/maps.md)。
- **看 V3 与 BR／TR**：从仓库根运行 `./sim_robots.sh --docker rc27_nav`；本机已有 Fortress 时直接运行 `./sim_robots.sh`。Docker 入口将资源复制到私有临时目录，退出后清理。该场景是 54 个场地／道具模型加 2 台 `video_v2` 原型，不启动 ROS／LIO／Nav2，默认不启 GPU Sensors。已实测 56 模型、1000 物理步，见 [地图文档](../docs/maps.md)。
- **omni4 导航仿真**：使用 Gazebo Classic 11，继续由 `sim_map.sh`／`sim_nav.sh` 装配，与 Fortress 双车场景分别运行。
- V3 草案通过 `./sim_map.sh field:=cn_v3_draft` 选择；默认 `legacy` 衔接旧先验图。换场地后重新建图。
- `worlds/`：场地 world、V3 单场地与双车场景。
- [robots/README.md](robots/README.md)：BR 麦轮滚子、TR 普通轮和机构模型、原生控制话题及生成方法；模型位于 `robots/models/robocon_{br,tr}_video_v2/`。
- Classic 导航启动：仓库根 `./sim_map.sh`、`./sim_nav.sh`、`./sim_teleop.sh`、`./sim_save_map.sh`。
- LIO：**FAST-LIVO2（无视觉）**，与真机共用 `config/livo_mid360.yaml`；
  抽稀节点也是真机那个 `livox_preprocess.py`（同一话题 `/livox/lidar_filtered`）。
- 重定位：`sim_navigation.launch.py` 走真机那条
  `teaser_gicp_node -> reloc_bridge -> /relocalization_result + map->odom`，
  LIO 以 `locate_in_prior_map` 跟踪先验图；旧的 4-DOF `prior_map_localizer`
  仍保留，用 `reloc_impl:=voxel` 切换。
- 雷达插件源码：`src/ros2_livox_simulation/`。
- 无 GPU 的机器跑仿真需要 Mesa 软件渲染，见 `docs/status.md` 第五节。
