#!/usr/bin/env bash
# 真机·存图：先验点云 + 轨迹 + 2D 代价地图 + meta.json
#    前提：./real_map.sh 还在跑（累积器在内存里攒着整张图）
#
#    ./real_save_map.sh --imu-height 0.30      # ★ IMU 离地高度要实测
#    ./real_save_map.sh --cloud maps/raw.pcd --traj maps/raw_traj.txt   # 离线重跑
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 私有 DDS 域(42) + 私有 Gazebo 端口(11355) + 全部环境依赖，说明见 env.sh
source "${ROOT}/env.sh"
echo "[环境] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"

exec "${ROOT}/Nav/real/scripts/save_prior_map.sh" "$@"
