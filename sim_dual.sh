#!/usr/bin/env bash
# ============================================================================
#  两套仿真 + 两套键盘遥控（同一台机器同时跑，互不干扰）
# ============================================================================
#  做法：每套用**不同的 ROS_DDS 域**和**不同的 Gazebo master 端口**，
#        键盘遥控各自连自己的域，所以能两个人同时开。
#
#      ./sim_dual.sh                 # 起两套建图仿真（Gazebo 窗口）
#      RC_RVIZ=true ./sim_dual.sh    # 两套都带 RViz（吃资源）
#      RC_GUI=false ./sim_dual.sh    # 两套都不开 Gazebo 窗口（远程/压测）
#      A_DOMAIN=42 B_DOMAIN=43 ./sim_dual.sh
#
#  起来之后**另开两个终端**分别遥控（必须带对应的域）：
#      RC_DOMAIN_ID=42 ./sim_teleop.sh     # 一号车
#      RC_DOMAIN_ID=43 ./sim_teleop.sh     # 二号车
#
#  存图也带域：RC_DOMAIN_ID=42 ./sim_save_map.sh car1
#  Ctrl-C 只停本脚本拉起的这两套（kill_sim.sh 按域清理，不会误杀别人的仿真）。
# ============================================================================
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

A_DOMAIN="${A_DOMAIN:-42}"
B_DOMAIN="${B_DOMAIN:-43}"
A_PORT="${A_PORT:-11355}"
B_PORT="${B_PORT:-11356}"
RC_GUI="${RC_GUI:-true}"
RC_RVIZ="${RC_RVIZ:-false}"
LAUNCH="${LAUNCH:-sim_map.sh}"     # 也可以 LAUNCH=sim_nav.sh

if [ "${A_DOMAIN}" = "${B_DOMAIN}" ]; then
    echo "A_DOMAIN 和 B_DOMAIN 不能相同（否则两套仿真会互相串）" >&2
    exit 2
fi
if [ "${A_PORT}" = "${B_PORT}" ]; then
    echo "A_PORT 和 B_PORT 不能相同（否则两个 gzserver 抢同一个 master）" >&2
    exit 2
fi

mkdir -p "${ROOT}/logs"
LOG_A="${ROOT}/logs/simA_d${A_DOMAIN}.log"
LOG_B="${ROOT}/logs/simB_d${B_DOMAIN}.log"

PIDS=()
start_one() {
    local name="$1" domain="$2" port="$3" log="$4"
    echo "[${name}] 域 ${domain} / Gazebo ${port} → ${log}"
    (
        cd "${ROOT}" || exit 1
        exec setsid env RC_DOMAIN_ID="${domain}" \
            RC_GAZEBO_MASTER_URI="http://127.0.0.1:${port}" \
            RC_GUI="${RC_GUI}" RC_RVIZ="${RC_RVIZ}" \
            bash "${ROOT}/${LAUNCH}" "${@:5}"
    ) > "${log}" 2>&1 &
    PIDS+=("$!")
}

wait_topic() {
    local domain="$1" topic="$2" total="${3:-90}"
    for _ in $(seq 1 "${total}"); do
        if ROS_DOMAIN_ID="${domain}" timeout 3 ros2 topic list 2>/dev/null \
                | grep -qx "${topic}"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

cleanup() {
    echo
    echo '[cleanup] 停掉两套仿真…'
    for d in "${A_DOMAIN}" "${B_DOMAIN}"; do
        ( cd "${ROOT}" && RC_DOMAIN_ID="${d}" bash -c \
              'source ./env.sh >/dev/null 2>&1; ros2 run rc_field_nav kill_sim.sh' \
              >/dev/null 2>&1 ) || true
    done
    for pid in "${PIDS[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
    sleep 2
    for pid in "${PIDS[@]}"; do kill -KILL -- "-${pid}" 2>/dev/null || true; done
    wait 2>/dev/null || true
    echo '[cleanup] 完成'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

source "${ROOT}/env.sh" >/dev/null 2>&1
start_one A "${A_DOMAIN}" "${A_PORT}" "${LOG_A}" "$@"
start_one B "${B_DOMAIN}" "${B_PORT}" "${LOG_B}" "$@"

echo '[1/2] 等一号车 /cmd_vel …'
wait_topic "${A_DOMAIN}" /cmd_vel 120 || { echo "一号车没起来，看 ${LOG_A}" >&2; exit 1; }
echo '[2/2] 等二号车 /cmd_vel …'
wait_topic "${B_DOMAIN}" /cmd_vel 120 || { echo "二号车没起来，看 ${LOG_B}" >&2; exit 1; }

cat <<EOF

==============================================================
 两套仿真都起来了
   一号车：ROS_DOMAIN_ID=${A_DOMAIN}   Gazebo http://127.0.0.1:${A_PORT}   日志 ${LOG_A}
   二号车：ROS_DOMAIN_ID=${B_DOMAIN}   Gazebo http://127.0.0.1:${B_PORT}   日志 ${LOG_B}

 另开两个终端遥控（★ 一定要带域，否则控不到）：
   RC_DOMAIN_ID=${A_DOMAIN} ./sim_teleop.sh
   RC_DOMAIN_ID=${B_DOMAIN} ./sim_teleop.sh

 存图：
   RC_DOMAIN_ID=${A_DOMAIN} ./sim_save_map.sh car1
   RC_DOMAIN_ID=${B_DOMAIN} ./sim_save_map.sh car2

 本终端 Ctrl-C 停掉两套。
==============================================================
EOF

wait
