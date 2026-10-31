# Nav —— 导航层（real / sim）

`real` 与 `sim` **同构**（M3 已完成）：同一套 LIO、同一套话题、同一份参数文件。

| | real | sim |
| --- | --- | --- |
| LiDAR | Mid-360（`/livox/lidar`，CustomMsg） | Gazebo 插件（同一话题、同一消息） |
| IMU | 雷达内置 IMU（`/livox/imu`） | Gazebo IMU 插件（同一话题） |
| 抽稀 | `Nav/real/scripts/livox_preprocess.py` | **同一个脚本** |
| LIO | FAST-LIVO2（无视觉）+ `Nav/config/livo_mid360.yaml` | **同左**（launch 覆盖外参 / sim time） |
| 定位修正 | 6-DOF 重定位（TEASER++ + VGICP）+ `reloc_bridge` | **同左** |
| 导航 | Nav2 | Nav2 |

统一话题：`/livox/lidar`、`/livox/lidar_filtered`、`/livox/imu`、`/aft_mapped_to_init`、
`/tf`、`/cmd_vel`、`/scan`、`/map`、`/cloud_registered`、`/Laser_map`、`/reloc_status`。

- 真机入口：仓库根 `real_*.sh` → `Nav/real/launch/`
- 仿真入口：仓库根 `sim_*.sh` → `Nav/ros2_ws/src/rc_field_nav/launch/`
- 共用参数：`Nav/config/`
- 世界/模型：`Nav/sim/worlds/`
