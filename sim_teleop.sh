#!/usr/bin/env bash
# ④ 键盘遥控（全向车：a/d 横移、q/e 自转）
#    必须在**真正的终端窗口**里跑（要读键盘），先起 ./sim_map.sh 或 ./sim_nav.sh
#
#    ./sim_teleop.sh                 # 交互遥控
#    ./sim_teleop.sh --speed 0.5     # 换速度档
#    ./sim_teleop.sh --seq "w:3 a:2" # 脚本化回放（不需要终端）
#    按键：w/s 前后 | a/d 左右 | q/e 自转 | 空格 急停 | z 归零 | t 退出
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 私有 DDS 域(42) + 私有 Gazebo 端口(11355) + 全部环境依赖，说明见 env.sh
source "${ROOT}/env.sh" || exit
echo "[环境] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"

exec ros2 run rc_field_nav omni_teleop.py "$@"
