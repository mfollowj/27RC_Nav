#!/usr/bin/env bash
# ============================================================================
#  小机器人默认启动：雷达驱动 + IMU 翻转 + 抽稀 + FAST-LIVO2 + mcu_bridge
# ============================================================================
#  取代原来的  ros2 launch sentry_bringup position_only.launch.py
#  （那套是 fast_lio + send_data；现在是本仓库的 fast_livo + 0x7E/0x21 位置帧）
#
#  用法（在机器人上）：
#      ./start_robot.sh                 # 正常启动
#      MCU_DRY_RUN=1 ./start_robot.sh   # 串口只打日志、不发字节（第一次上车建议）
#      RVIZ=1 ./start_robot.sh          # 顺便开 RViz（机器人有屏幕时）
#      DOMAIN=0 ./start_robot.sh        # 换 DDS 域（默认 42，见 env.sh）
#
#  链路：
#      livox_ros_driver2 (15 Hz) ──/livox/lidar──┐
#                                 ──/livox/imu ──┤（原始 IMU：加速度 +1 g z，直接喂 LIO）
#                                                v
#                     livox_preprocess -> fastlivo_mapping -> TF odom->base_link
#                                                │
#                     RViz(本机) /goal_pose -> mcu_bridge ──串口──> 单片机
#
#  说明：
#    * 本脚本**不跑** Nav2 / 6-DOF 重定位 —— 位姿由 MCU 闭环；Nav2 那套用
#      ./real_nav.sh（需要全场先验图）。
#    * 串口同一时刻只能有一个写者：启动前会把 SCURM 的 send_data 停掉。
#    * 每一步都**等数据真的通了**再往下走（驱动刚重启时 UDP 口可能还没让出来，
#      一次没数据会自动重启驱动重试）。
# ============================================================================
# 不要 set -u：ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES）
set +u

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"   # 解析 ~/start_robot.sh 软链
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"          # 仓库根

MCU_DRY_RUN="${MCU_DRY_RUN:-0}"
RVIZ="${RVIZ:-0}"
DOMAIN="${DOMAIN:-42}"

if [ -f "${ROOT}/env.sh" ]; then
    # shellcheck disable=SC1091
    source "${ROOT}/env.sh"
else
    echo "[error] 找不到 ${ROOT}/env.sh"
    exit 1
fi
export ROS_DOMAIN_ID="${DOMAIN}"   # env.sh 里可能又设一次，这里锁死

# ★ LIO 用哪一份 fast_livo：
#   优先本仓库自己编的（Nav/../Deps/livo_install，和 Nav/config/livo_mid360.yaml 配套，
#   帧名是 odom->base_link）；没有再退回环境里那份（SCURM/New_Nav，帧名 camera_init->aft_mapped）
LIO_WS="${ROOT}/Deps/livo_install"
if [ -f "${LIO_WS}/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${LIO_WS}/setup.bash" >/dev/null 2>&1
    LIO_FRAME="odom"
    LIO_DESC="$(ros2 pkg prefix fast_livo 2>/dev/null)"
else
    LIO_FRAME="camera_init"
    LIO_DESC="环境自带（帧名 camera_init）"
fi

echo "=============================================================="
echo " 小机器人启动 | ROS_DOMAIN_ID=${ROS_DOMAIN_ID} | MCU_DRY_RUN=${MCU_DRY_RUN}"
echo " LIO: ${LIO_DESC} | 帧: ${LIO_FRAME}"
echo " 仓库: ${ROOT}"
echo "=============================================================="

PIDS=()
_CLEANED=0

cleanup() {
    [ "${_CLEANED}" = "1" ] && return
    _CLEANED=1
    echo ""
    echo "[cleanup] 停止本次拉起的进程…"
    # 先 SIGINT（ros2 launch 收到 INT 才会把子节点一起关掉），再 TERM，最后 KILL
    for pid in "${PIDS[@]:-}"; do
        [ -n "${pid}" ] && kill -INT "${pid}" 2>/dev/null || true
    done
    sleep 3
    for pid in "${PIDS[@]:-}"; do
        [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null && kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]:-}"; do
        [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null && kill -KILL "${pid}" 2>/dev/null || true
    done
    # 兜底：launch 已经死了但节点还活着的情况（实测会残留 mcu_bridge/rot_imu）
    for pat in 'fastlivo_mappin[g]' 'livox_ros_driver2_nod[e]' 'rot_im[u]' \
               'livox_preproces[s]' 'mcu_bridge_nod[e]' 'livox_t[f]'; do
        pkill -f "${pat}" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 等某个话题真的有消息；$1=话题 $2=最多等几秒
wait_topic() {
    local topic="$1" total="${2:-30}" waited=0
    while [ "${waited}" -lt "${total}" ]; do
        if timeout 6 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
            echo "      ${topic} 有数据了（等了 ${waited}s）"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
    done
    return 1
}

DRIVER_LAUNCH="$(ros2 pkg prefix livox_ros_driver2 2>/dev/null)/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"

start_driver() {
    ros2 launch "${DRIVER_LAUNCH}" > /tmp/robot_driver.log 2>&1 &
    DRIVER_PID=$!
    PIDS+=(${DRIVER_PID})
}

# ---------------------------------------------------------------- 0) 停旧的
echo "[0/4] 停掉可能还在跑的旧栈（SCURM position_only / send_data / 旧驱动 / 旧 LIO）"
for pat in 'position_onl[y]' 'odom_serial_publishe[r]' 'livox_ros_driver2_nod[e]' \
           'fastlio_mappin[g]' 'fastlivo_mappin[g]' 'rot_im[u]' \
           'livox_preproces[s]' 'mcu_bridge_nod[e]' 'livox_t[f]'; do
    pkill -f "${pat}" 2>/dev/null || true
done
sleep 5          # 给驱动时间释放 UDP 口（太快重启会收不到雷达数据）

if fuser /dev/ttyUSB0 >/dev/null 2>&1; then
    echo "[warn] /dev/ttyUSB0 仍被占用："
    fuser -v /dev/ttyUSB0 2>&1 | sed 's/^/       /'
else
    echo "      /dev/ttyUSB0 空闲"
fi

# ---------------------------------------------------------------- 1) 驱动
if [ ! -f "${DRIVER_LAUNCH}" ]; then
    echo "[error] 找不到 livox 驱动 launch：${DRIVER_LAUNCH}"
    exit 1
fi
echo "[1/4] 雷达驱动（15 Hz，用机器人上已验证的 MID360_config.json）"
start_driver
if ! wait_topic /livox/lidar 40; then
    echo "[warn] 40 s 没收到 /livox/lidar，重启驱动一次（UDP 口没让干净时会这样）"
    kill "${DRIVER_PID}" 2>/dev/null || true
    pkill -f 'livox_ros_driver2_nod[e]' 2>/dev/null || true   # 兜底：launch 没带走节点时
    sleep 6
    start_driver
    if ! wait_topic /livox/lidar 40; then
        echo "[error] 雷达仍无数据，看 /tmp/robot_driver.log"
        tail -10 /tmp/robot_driver.log | sed 's/^/       /'
        exit 1
    fi
fi
wait_topic /livox/imu 20 || { echo "[error] /livox/imu 没有数据，LIO 起不来"; exit 1; }

# ---------------------------------------------------------------- 3) LIO
echo "[2/4] FAST-LIVO2（共用参数 Nav/config/livo_mid360.yaml，订阅原始 /livox/imu）"
ros2 launch "${ROOT}/Nav/real/launch/real_mapping.launch.py" \
     livox_driver:=false record_traj:=false map_accum:=false \
     rviz:="$( [ "${RVIZ}" = "1" ] && echo true || echo false )" \
     > /tmp/robot_lio.log 2>&1 &
PIDS+=($!)
if ! wait_topic /aft_mapped_to_init 45; then
    echo "[error] LIO 没出里程计，看 /tmp/robot_lio.log"
    tail -20 /tmp/robot_lio.log | sed 's/^/       /'
    exit 1
fi

# ---------------------------------------------------------------- 4) 串口桥
echo "[3/4] mcu_bridge（当前位姿 + RViz 目标 -> 串口 0x7E/0x21）"
# 机器人上这套 fast_livo 的帧名是 camera_init->aft_mapped（不是 odom->base_link），
# 所以 output_frame 用 camera_init；当前位姿走里程计话题 /aft_mapped_to_init
ros2 launch mcu_bridge mcu_bridge.launch.py \
     output_frame:="${LIO_FRAME}" \
     dry_run:="$( [ "${MCU_DRY_RUN}" = "1" ] && echo true || echo false )" \
     > /tmp/robot_mcu.log 2>&1 &
PIDS+=($!)
sleep 4

# ---------------------------------------------------------------- 5) 自检
echo "[4/4] 自检"
echo "--- 节点 ---"
ros2 node list 2>/dev/null | sed 's/^/      /'
echo "--- mcu_bridge 状态（cur 应该不是 NONE）---"
timeout 10 ros2 topic echo /mcu_bridge/status --once --field data 2>/dev/null | grep -v '^---' | sed 's/^/      /'
echo "--- TF odom->base_link ---"
timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -4 | sed 's/^/      /'

echo ""
echo "=============================================================="
echo " 起来了。日志：/tmp/robot_driver.log"
echo "            /tmp/robot_lio.log /tmp/robot_mcu.log"
echo " 本机 RViz（同域 ${ROS_DOMAIN_ID}，Fixed Frame 用 ${LIO_FRAME}）："
echo "   rviz2 -d Nav/real/rviz/real_nav.rviz"
echo " Ctrl-C 退出会停掉本次拉起的全部进程。"
echo "=============================================================="

wait
