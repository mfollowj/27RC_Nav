#!/usr/bin/env bash
# 仅同步 MCU 包并编译，不启动节点；保留远端已有 underlay prefix chain。
# MCU_HOST=192.168.10.10 MCU_USER=rc MCU_REPO=/home/rc/SCU27RC_Nav ./tools/deploy_mcu_bridge.sh
# --no-build 只同步。首次整仓部署请使用根 build.sh。
set -euo pipefail

HOST="${MCU_HOST:-192.168.10.10}"
USER_="${MCU_USER:-rc}"
REMOTE_REPO="${MCU_REPO:-/home/rc/SCU27RC_Nav}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_PKG="${ROOT}/src/mcu_bridge"
REMOTE_PKG="${REMOTE_REPO}/src/mcu_bridge"
BUILD=1
for a in "$@"; do
    case "${a}" in
        --no-build) BUILD=0 ;;
        -h|--help)
            echo '用法：MCU_HOST=<机器人地址> MCU_USER=rc MCU_REPO=<远端仓库路径> ./tools/deploy_mcu_bridge.sh [--no-build]'
            exit 0 ;;
        *) echo "未知参数：${a}" >&2; exit 2 ;;
    esac
done

SSH=(ssh -o BatchMode=yes -o ConnectTimeout=15 "${USER_}@${HOST}")
echo "[1/3] 同步 ${LOCAL_PKG}"
printf -v REMOTE_CMD 'mkdir -p -- %q' "${REMOTE_PKG}"
"${SSH[@]}" "${REMOTE_CMD}"
rsync -az --delete --protect-args \
    -e 'ssh -o BatchMode=yes -o ConnectTimeout=15' \
    --exclude '__pycache__' --exclude '*.pyc' --exclude '.pytest_cache' \
    "${LOCAL_PKG}/" "${USER_}@${HOST}:${REMOTE_PKG}/"

if [ "${BUILD}" = 0 ]; then
    echo '[2/3] 已跳过编译（--no-build）'
    exit 0
fi

echo '[2/3] 在远端按原 prefix chain 编译'
printf -v REMOTE_CMD 'bash -s -- %q' "${REMOTE_REPO}"
"${SSH[@]}" "${REMOTE_CMD}" <<'REMOTE'
set -e
set -o pipefail
repo="$1"
workspace="${repo}"
pkg="${workspace}/src/mcu_bridge"
setup_file="${workspace}/install/setup.bash"

read_chain() {
    python3 - "$1" <<'PY'
import pathlib
import re
import sys
path = pathlib.Path(sys.argv[1])
if path.is_file():
    for prefix in re.findall(r'COLCON_CURRENT_PREFIX="([^"]+)"', path.read_text()):
        if prefix.startswith('/'):
            print(prefix)
PY
}
mapfile -t chain < <(read_chain "${setup_file}")
printf '原 prefix chain: %s\n' "${chain[@]}"
source /opt/ros/humble/setup.bash
for prefix in "${chain[@]}"; do
    if [ -f "${prefix}/setup.bash" ]; then source "${prefix}/setup.bash"; fi
done
if [ -f "${setup_file}" ]; then
    cp -a "${setup_file}" /tmp/mcu_bridge_setup.bash.bak
fi
cd "${workspace}"
colcon build --base-paths "${workspace}/src" --packages-select mcu_bridge
source install/local_setup.bash

echo '[3/3] 包路径与既有协议测试'
ros2 pkg prefix mcu_bridge
python3 "${pkg}/test/test_frame.py"
echo 'prefix chain 差异（原 setup 备份在 /tmp/mcu_bridge_setup.bash.bak）：'
diff -u <(printf '%s\n' "${chain[@]}") <(read_chain "${setup_file}") || true
REMOTE

echo "完成。机器人上 source \"${REMOTE_REPO}/env.sh\" 后，可先用 MCU_DRY_RUN=1 ./start_robot.sh 联调。"
echo '串口必须只有一个写者；脚本没有启动驱动、串口桥或运动。'
