#!/usr/bin/env bash
# ① 仿真建图：Gazebo(physics 驱动 + Mid-360) + FAST-LIO 建图 + RViz
#    走完场地后，另开终端执行 ./sim_save_map.sh 存图。
#    键盘开车：./sim_teleop.sh
#
#    ./sim_map.sh                 # 默认开 Gazebo 窗口 + RViz
#    RC_GUI=false ./sim_map.sh    # 无界面（远程/压测）
#    ./sim_map.sh x:=-4.3 y:=1.5 yaw:=0.0    # 换出生点（★ 导航要用同一起点）
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 私有 DDS 域(42) + 私有 Gazebo 端口(11355) + 全部环境依赖，说明见 env.sh
source "${ROOT}/env.sh"
echo "[环境] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"

echo "[1/2] 清理上一次残留的仿真进程"
ros2 run rc_field_nav kill_sim.sh >/dev/null 2>&1 || true
echo "[2/2] 启动仿真建图（另开终端：./sim_teleop.sh 键盘开车；./sim_save_map.sh 存图）"
exec ros2 launch rc_field_nav lio_map.launch.py \
     gui:="${RC_GUI:-true}" rviz:="${RC_RVIZ:-true}" "$@"
