#!/usr/bin/env bash
# ============================================================================
#  同一个 Gazebo 里放两辆车 + 两个 RViz，各控各的
# ============================================================================
#  两辆车各自一个 ROS 命名空间：
#      car1: /car1/cmd_vel  /car1/odom  TF car1/odom -> car1/base_link
#      car2: /car2/cmd_vel  /car2/odom  TF car2/odom -> car2/base_link
#
#      ./sim_two.sh                       # Gazebo 界面 + 两个 RViz
#      ./sim_two.sh gui:=false            # 只开两个 RViz
#      ./sim_two.sh field:=cn_v3_draft    # V3 场地
#      ./sim_two.sh x1:=-4 y1:=1 x2:=-4 y2:=3
#
#  起来后另开两个终端遥控（--topic 指到对应车）：
#      ./sim_teleop.sh --topic /car1/cmd_vel
#      ./sim_teleop.sh --topic /car2/cmd_vel
#
#  两个 RViz 窗口是自动开的，Fixed Frame 分别是 car1/odom、car2/odom。
#  注意：双车用的是 planar_move 运动学底盘（轮子只做外观、不转），因为
#  同一 gzserver 里两套 gazebo_ros2_control 的 controller_manager 会互相干扰。
# ============================================================================
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/env.sh" || exit 1

echo "[两车] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "      遥控：./sim_teleop.sh --topic /car1/cmd_vel   （另一个终端）"
echo "            ./sim_teleop.sh --topic /car2/cmd_vel"
echo "      日志：logs/two_cars.log（本脚本不重定向，直接用终端输出）"

# 先清掉本域上一个仿真（按 ROS 域判定，不会误杀别人的 Gazebo）
ros2 run rc_field_nav kill_sim.sh >/dev/null 2>&1 || true

exec ros2 launch rc_field_nav sim_two_robots.launch.py \
    gui:="${RC_GUI:-true}" rviz:="${RC_RVIZ:-true}" "$@"
