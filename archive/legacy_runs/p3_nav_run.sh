#!/usr/bin/env bash
# ============================================================================
#  [一次性调试脚本] P3 验证②：仿真导航（TEASER++/VGICP + reloc_bridge + fast_livo + Nav2）
#  全部放在一个 shell 里跑，最后自动发一个 Nav2 目标并检查机器人是否真的动了。
# ============================================================================
set +u
ROOT="/home/zx/ITEM/SCU27RC_Nav"
LOG="${ROOT}/logs/p3_nav"
mkdir -p "${LOG}"

export HOME=/tmp/rc_p3_nav_home
export ROS_HOME="${HOME}/.ros"
export ROS_LOG_DIR="${HOME}/roslog"
mkdir -p "${HOME}" "${ROS_HOME}" "${ROS_LOG_DIR}"
# 沙箱：无 GPU 设备节点 + 离线，见 logs/p3_map_run.sh 的说明
export __GLX_VENDOR_LIBRARY_NAME=mesa
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_DATABASE_URI=""

source "${ROOT}/env.sh"
cd "${ROOT}"

MAPS="${ROOT}/Nav/ros2_ws/install/rc_field_nav/share/rc_field_nav/maps"
PRIOR=""
COST=""
for y in $(ls -t "${MAPS}"/*_costmap.yaml 2>/dev/null); do
    n="$(basename "${y}" _costmap.yaml)"
    if [ -s "${MAPS}/${n}_map.pcd" ]; then
        PRIOR="${MAPS}/${n}_map.pcd"; COST="${MAPS}/${n}_costmap.yaml"; break
    fi
done
echo "先验图: ${PRIOR}"
echo "2D 图 : ${COST}"

echo "[1/6] 清理残留"
ros2 run rc_field_nav kill_sim.sh >"${LOG}/kill1.log" 2>&1 || true
sleep 2

echo "[2/6] 起导航（headless，teaser 链路）"
RC_GUI=false RC_RVIZ=false ./sim_nav.sh \
  prior_map:="${PRIOR}" map:="${COST}" gui:=false rviz:=false \
  ${P3_EXTRA_ARGS:-} >"${LOG}/launch.log" 2>&1 &
LPID=$!
echo "launch pid=${LPID}"

# ---- 等 ROS 起来 ----
sleep 30
ros2 topic list >"${LOG}/topics.txt" 2>&1
ros2 node list >"${LOG}/nodes.txt" 2>&1

echo "[3/6] 等重定位收敛（最多 120 s）"
reloc_ok=0
for i in $(seq 1 24); do
    st="$(timeout 15 ros2 topic echo /reloc_status --once --field data 2>/dev/null | grep -v '^---' | tail -1)"
    echo "  ${i}: ${st:-<无>}"
    [ -n "${st}" ] && echo "${st}" >"${LOG}/reloc_status.txt"
    case "${st}" in
        *TRACKING*|*APPLIED*) reloc_ok=1; break ;;
    esac
    sleep 5
done
echo "reloc_ok=${reloc_ok}"

echo "[4/6] 检查 TF / LIO"
timeout 10 ros2 run tf2_ros tf2_echo map base_link >"${LOG}/tf_map_base.txt" 2>&1
echo "tf_map_base_rc=$?"
timeout 10 ros2 topic echo /aft_mapped_to_init --once >"${LOG}/odom_once.txt" 2>&1
echo "odom_rc=$?"
timeout 10 ros2 lifecycle get /bt_navigator >"${LOG}/bt_state.txt" 2>&1
cat "${LOG}/bt_state.txt"

echo "[4.5/6] 等 Nav2 激活（bt_navigator active 之前发目标会被拒）"
for i in $(seq 1 18); do
    btst="$(timeout 10 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)"
    echo "  bt: ${btst:-<无>}"
    case "${btst}" in
        active*) break ;;
    esac
    sleep 5
done

# 记录目标前的轮速里程计（/odom_gz，不依赖 LIO）
timeout 10 ros2 topic echo /odom_gz --once >"${LOG}/odom_gz_before.txt" 2>&1

echo "[5/6] 发 Nav2 目标"
# 目标点：先验图里 free 面积最大的连通域的重心（脚本里从 pgm 算，找不到就退 (1.0, 0.0)）
GOAL="$(python3 "${ROOT}/logs/p3_pick_goal.py" "${COST}" 2>/dev/null)"
[ -z "${GOAL}" ] && GOAL="1.0 0.0"
echo "目标 (map 系): ${GOAL}"
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $(echo ${GOAL} | cut -d' ' -f1), y: $(echo ${GOAL} | cut -d' ' -f2)}, orientation: {w: 1.0}}}}" \
  --feedback >"${LOG}/nav_goal.log" 2>&1
echo "goal_rc=$?"
tail -15 "${LOG}/nav_goal.log"

sleep 2
timeout 10 ros2 topic echo /odom_gz --once >"${LOG}/odom_gz_after.txt" 2>&1

echo "[6/6] 汇总"
echo "--- /reloc_status ---"; cat "${LOG}/reloc_status.txt" 2>/dev/null
echo "--- TF map->base_link ---"; head -8 "${LOG}/tf_map_base.txt" 2>/dev/null
echo "--- bt_navigator ---"; cat "${LOG}/bt_state.txt" 2>/dev/null
echo "--- odom_gz before/after ---"
grep -A3 'position' "${LOG}/odom_gz_before.txt" 2>/dev/null | head -5
grep -A3 'position' "${LOG}/odom_gz_after.txt" 2>/dev/null | head -5
echo "--- bridge/teaser 关键行 ---"
grep -E "reloc_bridge|teaser|TEASER|FIRST|RECOVER|播种" "${LOG}/launch.log" | tail -25

sleep 1
kill -INT ${LPID} 2>/dev/null
sleep 8
kill -TERM ${LPID} 2>/dev/null
ros2 run rc_field_nav kill_sim.sh >"${LOG}/kill2.log" 2>&1 || true
echo "=== 完 ==="
