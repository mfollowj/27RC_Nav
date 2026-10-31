#!/usr/bin/env bash
set -euo pipefail
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
ROOT="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
if [ "${1:-}" = --help ] || [ "${1:-}" = -h ]; then
    echo '用法：./sim_robots.sh [--docker <正在运行的容器>] [ign gazebo 参数]'
    echo '示例：./sim_robots.sh --docker rc27_nav -s --iterations 10'
    echo '只启动 Fortress BR/TR 场景，不启动 ROS、LIO 或 Nav2。'
    exit 0
fi
if [ "${1:-}" != --docker ]; then
    exec bash "${ROOT}/simulation/worlds/cn_v3_draft/start_fortress_robots.sh" "$@"
fi
if [ "$#" -lt 2 ]; then
    echo '--docker 后需要一个正在运行的容器名称。' >&2
    exit 2
fi
CONTAINER="$2"
shift 2
SCENE_DIR="$(docker exec "${CONTAINER}" mktemp -d /tmp/scu27rc_v3_robots.XXXXXX)"

cleanup() {
    docker exec "${CONTAINER}" bash -c '
        if [ -f "$1/scene.pid" ]; then
            read -r pid < "$1/scene.pid"
            if kill -0 -- "-${pid}" 2>/dev/null; then
                kill -INT -- "-${pid}" 2>/dev/null || true
                sleep 1
                kill -TERM -- "-${pid}" 2>/dev/null || true
                sleep 1
                kill -KILL -- "-${pid}" 2>/dev/null || true
            fi
        fi
        rm -rf -- "$1"
    ' bash "${SCENE_DIR}" >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[scene] ${CONTAINER}:${SCENE_DIR}"
docker exec "${CONTAINER}" mkdir -p "${SCENE_DIR}/simulation/worlds"
docker cp "${ROOT}/simulation/worlds/cn_v3_draft" \
    "${CONTAINER}:${SCENE_DIR}/simulation/worlds/"
docker cp "${ROOT}/simulation/robots" "${CONTAINER}:${SCENE_DIR}/simulation/"
# A separate session lets cleanup stop only this foreground scene, including its GUI.
docker exec -e "IGN_PARTITION=${IGN_PARTITION:-scu27rc_v3_robots_${USER:-user}_$$}" \
    "${CONTAINER}" setsid --wait bash -c '
        printf "%s\n" "$$" > "$1/scene.pid"
        exec bash "$1/simulation/worlds/cn_v3_draft/start_fortress_robots.sh" "${@:2}"
    ' bash "${SCENE_DIR}" "$@"
