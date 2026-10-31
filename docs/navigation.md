# 导航层：真机与仿真

真机与仿真共用 FAST-LIVO2 参数、点云处理和 TEASER++／VGICP 重定位实现。真机运行资源位于 `robot/`，仿真装配位于 `src/rc_field_nav/`，场地源位于 `simulation/`。

| 环节 | 真机 | 仿真 |
| --- | --- | --- |
| 点云与 IMU | Mid-360 驱动 | Classic 雷达／IMU 插件 |
| 抽稀 | `robot/scripts/livox_preprocess.py` | 同一脚本 |
| LIO | FAST-LIVO2 无视觉模式，`config/livo_mid360.yaml` | 同一配置，launch 覆盖外参与仿真时钟 |
| 全局定位 | TEASER++／VGICP + `reloc_bridge` | 同一实现 |
| 导航 | 真机 Nav2 参数 | 仿真 Nav2 参数与轮速执行 |

主要数据为 `/livox/lidar`、`/livox/lidar_filtered`、`/livox/imu`、`/aft_mapped_to_init`、`/cloud_registered`、`/reloc_status`、TF 和 `/cmd_vel`。完整链路与发布者见 [源码索引](source_index.md)。

- 真机建图：`real_map.sh` → `robot/launch/real_mapping.launch.py`。
- 真机重定位：`real_reloc.sh` → `robot/launch/real_relocalization.launch.py`。
- 仿真建图：`sim_map.sh` → `src/rc_field_nav/launch/sim_mapping.launch.py`。
- 仿真导航：`sim_nav.sh` → `src/rc_field_nav/launch/sim_navigation.launch.py`。
- MCU 位置闭环：`start_robot.sh`，独立于 Nav2 速度执行方式。

运行步骤见 [部署说明](getting_started.md)；V3 场地、地图配对与验证范围见 [地图说明](maps.md)；历史实测与待补充的速度反馈见 [当前状态](status.md)。
