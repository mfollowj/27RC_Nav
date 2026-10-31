#!/usr/bin/env bash
# ============================================================================
#  [一次性调试脚本] P3 验证①：仿真建图（fast_livo）-> 自动走场地 -> 存图
#  整个流程放在一个 shell 里跑，避免跨调用 ROS 发现不可靠的问题。
# ============================================================================
set +u
ROOT="/home/zx/ITEM/SCU27RC_Nav"
LOG="${ROOT}/logs/p3_map"
mkdir -p "${LOG}"

# 沙箱里 $HOME 只读：gazebo/ROS 都要写 ~/.gazebo 与 ~/.ros，挪到可写目录
export HOME=/tmp/rc_p3_home
export ROS_HOME="${HOME}/.ros"
export ROS_LOG_DIR="${HOME}/roslog"
mkdir -p "${HOME}" "${ROS_HOME}" "${ROS_LOG_DIR}"

# 沙箱里没有 /dev/dri、也没有 NVIDIA 设备节点：GLX 默认走 nvidia 会失败
# （BadValue on X_GLXCreateContext），强制走 Mesa 软件渲染；
# 同时关掉 gazebo 的在线模型库（离线时会卡在 "Getting models from ..."）
export __GLX_VENDOR_LIBRARY_NAME=mesa
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_DATABASE_URI=""

source "${ROOT}/env.sh"
cd "${ROOT}"

echo "[1/6] 清理残留"
ros2 run rc_field_nav kill_sim.sh >"${LOG}/kill1.log" 2>&1 || true
sleep 2

echo "[2/6] 起仿真建图（headless）"
RC_GUI=false RC_RVIZ=false \
  ./sim_map.sh gui:=false rviz:=false >"${LOG}/launch.log" 2>&1 &
LPID=$!
echo "launch pid=${LPID}"

# ---- 等雷达出数据（最多 90 s）----
ok=0
for i in $(seq 1 18); do
  sleep 5
  if timeout 8 ros2 topic echo /livox/lidar --once >/dev/null 2>&1; then ok=1; break; fi
  echo "  等待 /livox/lidar … ${i}"
done
echo "[3/6] 雷达数据=${ok}"
ros2 topic list >"${LOG}/topics_before.txt" 2>&1
ros2 node list >"${LOG}/nodes_before.txt" 2>&1
if [ "${ok}" != "1" ]; then
  echo "[中止] 雷达没数据，见 ${LOG}/launch.log"
  kill ${LPID} 2>/dev/null
  ros2 run rc_field_nav kill_sim.sh >/dev/null 2>&1 || true
  exit 1
fi

# 抽稀节点是否起来了（与真机同一个节点）
timeout 8 ros2 topic echo /livox/lidar_filtered --once >"${LOG}/filtered_once.txt" 2>&1
echo "filtered_ok=$?"

echo "[4/6] 自动走场地"
python3 "${ROOT}/logs/p3_sim_drive.py" --speed 0.45 >"${LOG}/drive.log" 2>&1
echo "drive_rc=$?"

sleep 2
echo "[5/6] 诊断"
timeout 10 ros2 topic echo /aft_mapped_to_init --once >"${LOG}/odom_once.txt" 2>&1
echo "odom_rc=$?"
timeout 8 ros2 topic echo /cloud_registered --once >"${LOG}/cloud_once.txt" 2>&1
echo "cloud_rc=$?"
ros2 topic list >"${LOG}/topics_after.txt" 2>&1
timeout 6 ros2 run tf2_ros tf2_echo odom base_link >"${LOG}/tf_odom_base.txt" 2>&1
timeout 6 ros2 run tf2_ros tf2_echo base_link livox_frame >"${LOG}/tf_base_livox.txt" 2>&1

echo "[6/6] 存图"
./sim_save_map.sh field >"${LOG}/save.log" 2>&1
echo "save_rc=$?"

sleep 1
kill -INT ${LPID} 2>/dev/null
sleep 6
kill -TERM ${LPID} 2>/dev/null
ros2 run rc_field_nav kill_sim.sh >"${LOG}/kill2.log" 2>&1 || true

echo "=== 完 ==="
echo "--- save.log ---"
tail -25 "${LOG}/save.log"
echo "--- maps ---"
ls -la "${ROOT}/Nav/ros2_ws/src/rc_field_nav/maps" 2>&1
