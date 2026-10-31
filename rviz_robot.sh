#!/usr/bin/env bash
# ============================================================================
#  本机 RViz（只看机器人那套 odom 系的栈）—— Fixed Frame 已固定成 odom
# ============================================================================
#  为什么单独一个脚本：real_nav.rviz 的 Fixed Frame 是 map（给带重定位/Nav2 的
#  map 系链路用的），机器人现在的栈只发 odom->base_link，用 real_nav.rviz 会
#  刷 "Message Filter dropping message: frame 'odom' ... queue is full"，
#  看起来就是"odom 没有数据"。
#
#    ./rviz_robot.sh              # 默认 Domain 42
#    DOMAIN=0 ./rviz_robot.sh     # 换域
# ============================================================================
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${DOMAIN:-42}"
if [ -f "${ROOT}/env.sh" ]; then
    source "${ROOT}/env.sh"
    export ROS_DOMAIN_ID="${DOMAIN:-42}"
fi

CFG="${ROOT}/Nav/real/rviz/robot_odom.rviz"
[ -f "${CFG}" ] || { echo "找不到 ${CFG}"; exit 1; }

echo "[rviz] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  Fixed Frame=odom  config=${CFG}"
exec rviz2 -d "${CFG}"
