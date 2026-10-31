#!/usr/bin/env bash
# 雷达 + 抽稀 + FAST-LIVO2 + MCU 位置桥；不启动 Nav2 或全局重定位。
# MCU_DRY_RUN=1 ./start_robot.sh：不开串口；RVIZ=1：打开 odom 系 RViz。
# DOMAIN/RC_DOMAIN_ID 可换 ROS 域，MCU_PORT 可换串口；标定来自 config/。
set -e
set +u

if [ "${1:-}" = --help ] || [ "${1:-}" = -h ]; then
    echo '用法：MCU_DRY_RUN=1 RVIZ=0 RC_DOMAIN_ID=42 ./start_robot.sh'
    echo '可选：DOMAIN（兼容旧用法）、MCU_PORT（默认 /dev/ttyUSB0）'
    exit 0
fi
if [ "$#" -gt 0 ]; then
    echo '未知参数；请用 --help 查看用法。串口空跑使用 MCU_DRY_RUN=1。' >&2
    exit 2
fi

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
ROOT="$(cd "$(dirname "${SCRIPT_PATH}")/.." && pwd)"
MCU_DRY_RUN="${MCU_DRY_RUN:-0}"
MCU_PORT="${MCU_PORT:-/dev/ttyUSB0}"
RVIZ="${RVIZ:-0}"
export RC_DOMAIN_ID="${DOMAIN:-${RC_DOMAIN_ID:-42}}"
source "${ROOT}/env.sh"

# 该配置与仓库内支持 publish.*_frame_id 的 FAST-LIVO2 配套。
LIO_PREFIX="$(ros2 pkg prefix fast_livo)"
case "${LIO_PREFIX}" in
    "${RC_DEPS_INSTALL}"|"${RC_DEPS_INSTALL}"/*) ;;
    *) echo '[error] fast_livo 不是本仓库构建产物，请先执行 ./build.sh robot。' >&2; exit 1 ;;
esac
ros2 pkg prefix livox_ros_driver2 >/dev/null
ros2 pkg prefix mcu_bridge >/dev/null

# 原脚本全局 pkill 会终止其他 ROS 域的仿真；保留单栈要求，改为指出冲突。
if pgrep -f 'position_onl[y]|odom_serial_publishe[r]|livox_ros_driver2_nod[e]|fastlio_mappin[g]|fastlivo_mappin[g]|mcu_bridge_nod[e]' >/dev/null; then
    echo '[error] 已有雷达、LIO 或串口写者。先停止对应实例后再启动：' >&2
    pgrep -af 'position_onl[y]|odom_serial_publishe[r]|livox_ros_driver2_nod[e]|fastlio_mappin[g]|fastlivo_mappin[g]|mcu_bridge_nod[e]' >&2
    exit 1
fi
if [ "${MCU_DRY_RUN}" != 1 ] && fuser "${MCU_PORT}" >/dev/null 2>&1; then
    echo "[error] ${MCU_PORT} 已被占用，串口必须只有一个写者。" >&2
    fuser -v "${MCU_PORT}" >&2
    exit 1
fi

echo "小机器人启动 | ROS_DOMAIN_ID=${ROS_DOMAIN_ID} | MCU_DRY_RUN=${MCU_DRY_RUN}"
echo "FAST-LIVO2: ${LIO_PREFIX} | TF: odom -> base_link"

PIDS=()
_CLEANED=0
cleanup() {
    [ "${_CLEANED}" = 1 ] && return
    _CLEANED=1
    echo '[cleanup] 停止本次启动的进程组…'
    for pid in "${PIDS[@]}"; do kill -INT -- "-${pid}" 2>/dev/null || true; done
    sleep 3
    for pid in "${PIDS[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
    sleep 2
    for pid in "${PIDS[@]}"; do kill -KILL -- "-${pid}" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_topic() {
    local topic="$1" total="${2:-30}"
    if timeout "${total}" ros2 topic echo "${topic}" --once \
            --qos-reliability best_effort >/dev/null 2>&1; then
        echo "      ${topic} 已收到数据"
        return 0
    fi
    return 1
}

start_driver() {
    # 使用本仓库的网络配置入口，保留其按网卡选择 host IP 的行为。
    setsid ros2 launch "${ROOT}/robot/launch/real_livox_driver.launch.py" \
        > /tmp/robot_driver.log 2>&1 &
    DRIVER_PID=$!
    PIDS+=("${DRIVER_PID}")
}

echo '[1/4] Livox 驱动'
start_driver
if ! wait_topic /livox/lidar 40; then
    echo '[warn] 40秒内无雷达数据，停止本次驱动并重试一次。'
    kill -INT -- "-${DRIVER_PID}" 2>/dev/null || true
    sleep 6
    kill -TERM -- "-${DRIVER_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL -- "-${DRIVER_PID}" 2>/dev/null || true
    wait "${DRIVER_PID}" 2>/dev/null || true
    PIDS=()
    start_driver
    if ! wait_topic /livox/lidar 40; then
        tail -n 10 /tmp/robot_driver.log >&2
        echo '[error] 雷达仍无数据，检查网卡与 robot/config/mid360_config.json。' >&2
        exit 1
    fi
fi
wait_topic /livox/imu 20 || { echo '[error] /livox/imu 无数据。' >&2; exit 1; }

echo '[2/4] FAST-LIVO2（原始 /livox/imu，config/livo_mid360.yaml）'
setsid ros2 launch "${ROOT}/robot/launch/real_mapping.launch.py" \
    livox_driver:=false record_traj:=false map_accum:=false \
    rviz:="$( [ "${RVIZ}" = 1 ] && echo true || echo false )" \
    > /tmp/robot_lio.log 2>&1 &
PIDS+=("$!")
if ! wait_topic /aft_mapped_to_init 45; then
    tail -n 20 /tmp/robot_lio.log >&2
    echo '[error] FAST-LIVO2 未输出里程计。' >&2
    exit 1
fi

echo '[3/4] MCU 位置桥'
setsid ros2 launch mcu_bridge mcu_bridge.launch.py \
    output_frame:=odom base_frame:=base_link port:="${MCU_PORT}" \
    dry_run:="$( [ "${MCU_DRY_RUN}" = 1 ] && echo true || echo false )" \
    > /tmp/robot_mcu.log 2>&1 &
PIDS+=("$!")

echo '[4/4] 状态（当前位姿不应为 NONE）'
timeout 15 ros2 topic echo /mcu_bridge/status --once --field data
echo '日志：/tmp/robot_driver.log /tmp/robot_lio.log /tmp/robot_mcu.log'
echo "本机可运行 ./rviz_robot.sh；Fixed Frame=odom，ROS_DOMAIN_ID=${ROS_DOMAIN_ID}。"
echo 'Ctrl-C 停止本次启动的全部进程。'

if wait -n "${PIDS[@]}"; then status=1; else status=$?; fi
echo '[error] 一个启动进程已退出，正在关闭本次其余进程；请查看对应日志。' >&2
exit "${status}"
