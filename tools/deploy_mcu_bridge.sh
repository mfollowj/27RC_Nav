#!/usr/bin/env bash
# ============================================================================
#  把本机的 mcu_bridge 包同步到机器人小电脑并编译
# ============================================================================
#  只同步 Nav/ros2_ws/src/mcu_bridge 这一个包，不动机器人上其它代码。
#
#  ★ 为什么要"保持原有的 prefix chain"：
#     机器人上 Nav/ros2_ws/install/setup.bash 里 chain 了
#     /home/rc/New_Nav/install、/home/rc/New_Nav/SCURC_Nav_Sim/install 等
#     （这些不在 env.sh 里）。colcon build 会**重新生成**这个 setup.bash，
#     如果编译时没 source 同样的前缀，它们就会从 chain 里消失，影响机器人上
#     别的包。所以本脚本先从远端 setup.bash 里把现有 chain 抽出来，
#     按同样顺序 source 之后再编译，并对比改动。
#
#  用法：
#      ./tools/deploy_mcu_bridge.sh                    # 默认 192.168.10.10
#      MCU_HOST=192.168.10.10 ./tools/deploy_mcu_bridge.sh
#      ./tools/deploy_mcu_bridge.sh --no-build         # 只同步不编译
# ============================================================================
set -euo pipefail

HOST="${MCU_HOST:-192.168.10.10}"
USER_="${MCU_USER:-rc}"
REMOTE_REPO="${MCU_REPO:-/home/rc/SCU27RC_Nav}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_PKG="${ROOT}/Nav/ros2_ws/src/mcu_bridge"
REMOTE_PKG="${REMOTE_REPO}/Nav/ros2_ws/src/mcu_bridge"
REMOTE_WS="${REMOTE_REPO}/Nav/ros2_ws"

BUILD=1
for a in "$@"; do
    [ "$a" = "--no-build" ] && BUILD=0
done

[ -d "${LOCAL_PKG}" ] || { echo "找不到 ${LOCAL_PKG}"; exit 1; }

SSH=(ssh -o BatchMode=yes -o ConnectTimeout=15 "${USER_}@${HOST}")

echo "[1/4] 同步 ${LOCAL_PKG}"
"${SSH[@]}" "mkdir -p '${REMOTE_PKG}'"
rsync -az --delete \
      --exclude '__pycache__' --exclude '*.pyc' --exclude '.pytest_cache' \
      "${LOCAL_PKG}/" "${USER_}@${HOST}:${REMOTE_PKG}/"

if [ "${BUILD}" = "0" ]; then
    echo "[2/4] 跳过编译（--no-build）"
    exit 0
fi

echo "[2/4] 读取远端现有 prefix chain（编译后要原样保留）"
CHAIN="$("${SSH[@]}" "cat '${REMOTE_WS}/install/setup.bash' 2>/dev/null" \
    | python3 -c 'import re,sys
t = sys.stdin.read()
print(" ".join(p for p in re.findall(r"COLCON_CURRENT_PREFIX=\"([^\"]+)\"", t)
              if p.startswith("/")))' || true)"
echo "      chain: ${CHAIN:-<空>}"

echo "[3/4] 远端编译"
REMOTE_SCRIPT="$(cat <<EOF
set -e
source /opt/ros/humble/setup.bash
for p in ${CHAIN}; do
  if [ -f "\$p/setup.bash" ]; then source "\$p/setup.bash" >/dev/null 2>&1; fi
done
cp -a '${REMOTE_WS}/install/setup.bash' '/tmp/mcu_bridge_setup.bash.bak' 2>/dev/null || true
cd '${REMOTE_WS}'
colcon build --packages-select mcu_bridge
source install/setup.bash
echo ''
echo '[4/4] 校验'
ros2 pkg prefix mcu_bridge
python3 '${REMOTE_PKG}/test/test_frame.py' | tail -2
echo '--- 新的 chain（应与上面一致）---'
grep -o 'COLCON_CURRENT_PREFIX="[^"]*"' install/setup.bash | sed 's/.*="//;s/"\$//'
EOF
)"

"${SSH[@]}" bash -s <<<"${REMOTE_SCRIPT}"

echo
echo "完成。机器人上的用法："
echo "  ssh ${USER_}@${HOST}"
echo "  source ${REMOTE_REPO}/env.sh"
echo "  ros2 launch mcu_bridge mcu_bridge.launch.py dry_run:=true   # 先空跑看帧"
echo "  ros2 launch mcu_bridge mcu_bridge.launch.py                 # 真发串口"
echo
echo "注意：/dev/ttyUSB0 同一时刻只能有一个写者。切换前先停掉 send_data："
echo "  ssh ${USER_}@${HOST} 'pkill -f odom_serial_publisher'"
