#!/usr/bin/env bash
# 在仓库当前位置编译；不启动驱动、仿真或串口。
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-all}"
if [ "$#" -gt 0 ]; then shift; fi
case "${MODE}" in
    all|robot|deps|nav) ;;
    -h|--help)
        echo '用法：./build.sh [all|robot|deps|nav] [colcon 参数]'
        echo '  all（默认）：依赖 + 完整导航工作区'
        echo '  robot：真机依赖 + relocalization/mcu_bridge'
        echo '         跳过 fast_lio 对照包及仿真资源/Gazebo 插件'
        echo '  deps：只编依赖；nav：只编导航工作区（依赖须已编好）'
        echo '  JOBS=1 ./build.sh robot  # 内存较少的小电脑'
        exit 0 ;;
    *) echo "未知构建模式：${MODE}（用 --help 查看支持项）" >&2; exit 2 ;;
esac
export JOBS="${JOBS:-2}"

if [ "${MODE}" != nav ]; then
    DEPS_MODE=all
    if [ "${MODE}" = robot ]; then DEPS_MODE=robot; fi
    bash "${ROOT}/third_party/build_deps.sh" "${DEPS_MODE}"
fi
if [ "${MODE}" = deps ]; then exit 0; fi

source "${ROOT}/env.sh"
cd "${ROOT}"
EXTRA=()
if [ "${MODE}" = robot ]; then
    EXTRA+=(--packages-ignore ros2_livox_simulation rc_field_nav)
fi
CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}" MAKEFLAGS="-j${JOBS}" \
    colcon build --executor sequential --base-paths "${ROOT}/src" \
    --build-base "${RC_WS_BUILD:-${ROOT}/build}" \
    --install-base "${RC_WS_INSTALL}" \
    "${EXTRA[@]}" "$@" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
echo "构建完成。当前终端执行：source \"${ROOT}/env.sh\""
