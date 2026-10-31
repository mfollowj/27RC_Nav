#!/usr/bin/env bash
# ============================================================================
#  编译 Deps/ 下的非官方依赖 -> Deps/install
# ============================================================================
#  前置（宿主提供，不在本目录）：
#    * ROS 2 Humble + Gazebo Classic 11 + Nav2（apt）
#    * libpcl-dev、libeigen3-dev、libomp-dev
#    * Livox-SDK2（源码装到 /usr/local；livox_ros_driver2 的硬依赖）
#
#  用法：
#      ./Deps/build_deps.sh                 # 全量
#      ./Deps/build_deps.sh small_gicp      # 只编某个（small_gicp/teaser/vikit/livox/fast_livo/fast_lio）
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL="${RC_DEPS_INSTALL:-${HERE}/install}"
JOBS="${JOBS:-4}"
mkdir -p "${INSTALL}"

source /opt/ros/humble/setup.bash

build_cmake() {   # $1=源码目录  $2=额外 cmake 参数
    local src="$1"; shift
    echo "==> cmake build ${src}"
    cmake -S "${src}" -B "${src}/build" \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX="${INSTALL}" \
          -DBUILD_TESTING=OFF \
          -DBUILD_TESTS=OFF \
          "$@"
    cmake --build "${src}/build" -j "${JOBS}"
    cmake --install "${src}/build"
}

build_ros() {     # $1..=包名
    echo "==> colcon build $*"
    ( cd "${HERE}" && \
      CMAKE_PREFIX_PATH="${INSTALL}:${CMAKE_PREFIX_PATH:-}" \
      colcon build --merge-install --install-base "${INSTALL}" \
                   --parallel-workers "${JOBS}" \
                   --packages-select "$@" \
                   --cmake-args -DCMAKE_BUILD_TYPE=Release \
                                -DBUILD_TESTING=OFF -DHUMBLE_ROS=humble -DROS_EDITION=ROS2 )
}

TARGET="${1:-all}"

if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "small_gicp" ]; then
    build_cmake "${HERE}/small_gicp"
fi
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "teaser" ]; then
    # TEASER++ 默认用 FetchContent 从 GitHub 拉 pmc/tinyply/spectra。
    # 离线/内网机器：先把这三个目录放到 TEASER-plusplus/third_party/<name>，
    # 脚本会自动改成从本地取（FETCHCONTENT_SOURCE_DIR_<NAME>）。
    TEASER_EXTRA=()
    for n in pmc tinyply spectra; do
        d="${HERE}/TEASER-plusplus/third_party/${n}"
        [ -d "${d}" ] && TEASER_EXTRA+=("-DFETCHCONTENT_SOURCE_DIR_${n^^}=${d}")
    done
    build_cmake "${HERE}/TEASER-plusplus" -DBUILD_TEASER_FPFH=ON "${TEASER_EXTRA[@]}"
fi
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "vikit" ]; then
    build_ros vikit_common vikit_ros vikit_py
fi
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "livox" ]; then
    build_ros livox_ros_driver2
fi
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "fast_livo" ]; then
    build_ros fast_livo
fi
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "fast_lio" ]; then
    build_ros fast_lio
fi

echo
echo "完成。产物在 ${INSTALL}"
echo "之后 source ${INSTALL}/setup.bash 即可（env.sh 已自动处理）。"
