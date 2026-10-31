# SCU27RC_Nav

2027 ROBOCON 导航与决策工程：**一套 Nav，两个数据源（real / sim），一个决策层（Decide）**。

- 真机：Mid-360（LiDAR + 内置 IMU）→ FAST-LIVO2（无视觉）→ 6-DOF 重定位 → Nav2
- 仿真：Gazebo（真实参数的 BR）→ **与真机同一套话题、同一个 LIO、同一份参数** → Nav2
- 决策层 `Decide/`：**框架设计与可行性分析已完成**（文档），代码未开始
- 依赖全部随仓库编译到 `Deps/install`，**不再依赖任何外部工作区**

## 目录

```
SCU27RC_Nav/
├── Decide/     # 决策层（框架 + 可行性文档；代码未开始）
├── Nav/        # config（real/sim 共用参数）/ real / sim / ros2_ws
├── Deps/       # 非官方依赖源码 + build_deps.sh（产物在 Deps/install）
├── Docs/       # 项目结构 / 一键启动脚本目录 / 项目当前情况
├── logs/       # 运行日志与一次性调试脚本
├── env.sh      # 统一环境（ROS_DOMAIN_ID=42、Gazebo 11355、Deps + Nav 工作区）
└── sim_*.sh / real_*.sh / check_lidar.sh
```

## 快速开始

```bash
source env.sh            # 手工敲 ros2/gz 命令前必做

# 首次换机器先编译依赖（约 3~5 min，TEASER++ 首次需要联网）
./Deps/build_deps.sh

# 仿真：建图 -> 存图 -> 导航
./sim_map.sh
./sim_save_map.sh field
./sim_nav.sh

# 真机：建图 -> 存图（注意单位是米）-> 导航
./real_map.sh
./real_save_map.sh --imu-height 0.30
./real_nav.sh teaser_inlier:=15 map_voxel:=0.2
```

## 文档

- [项目结构](Docs/项目结构.md)
- [一键启动脚本目录](Docs/一键启动脚本目录.md)
- [项目当前情况](Docs/项目当前情况.md)

### 决策层（Decide）

- [Decide 索引与结论](Decide/README.md)
- [01 框架设计](Decide/01-框架设计.md)
- [02 接口契约](Decide/02-接口契约.md)
- [03 策略落地映射](Decide/03-策略落地映射.md)
- [04 可行性分析](Decide/04-可行性分析.md)

> 兼容软链：`/home/zx/ITEM/RC_ground_sim -> SCU27RC_Nav`，全链验证后可删除。
