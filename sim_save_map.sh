#!/usr/bin/env bash
# ② 仿真存图：把当前建图结果落盘成先验地图三件套
#    前提：./sim_map.sh 还在跑，且已经用键盘把场地走过一遍
#
#    ./sim_save_map.sh            # -> maps/prior_map.pcd + prior_traj.txt + prior_costmap.*
#    ./sim_save_map.sh mymap      # 自定义名字
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 私有 DDS 域(42) + 私有 Gazebo 端口(11355) + 全部环境依赖，说明见 env.sh
source "${ROOT}/env.sh" || exit
echo "[环境] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"

exec ros2 run rc_field_nav save_prior_map.sh "$@"
