#!/usr/bin/env bash
# ============================================================================
#  编译 third_party/ 下的非官方依赖 -> third_party/install
# ============================================================================
#  前置（宿主提供，不在本目录）：
#    * ROS 2 Humble；真机导航需 Nav2，仿真另需 Gazebo Classic 11（apt）
#    * libpcl-dev、libeigen3-dev、libomp-dev
#    * Livox-SDK2（源码装到 /usr/local；livox_ros_driver2 的硬依赖）
#
#  用法：
#      ./third_party/build_deps.sh                 # 全量
#      ./third_party/build_deps.sh robot          # 真机依赖（不编保留对照包 fast_lio）
#      ./third_party/build_deps.sh small_gicp      # 只编某个组件
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_ROOT="${RC_DEPS_BUILD:-${HERE}/build}"
JOBS="${JOBS:-2}"
TARGET="${1:-all}"
case "${TARGET}" in
    all|robot|small_gicp|teaser|vikit|livox|fast_livo|fast_lio) ;;
    -h|--help)
        echo '用法：./third_party/build_deps.sh [all|robot|small_gicp|teaser|vikit|livox|fast_livo|fast_lio]'
        exit 0 ;;
    *) echo "未知依赖：${TARGET}（用 --help 查看支持项）" >&2; exit 2 ;;
esac
BUILD_FAST_LIO=1
if [ "${TARGET}" = robot ]; then
    TARGET=all
    BUILD_FAST_LIO=0
fi
source "${HERE}/../env.sh"
INSTALL="${RC_DEPS_INSTALL}"
mkdir -p "${INSTALL}"

build_cmake() {   # $1=源码目录  $2=额外 cmake 参数
    local src="$1"; shift
    local build_dir="${BUILD_ROOT}/$(basename "${src}")"
    echo "==> cmake build ${src}"
    cmake -S "${src}" -B "${build_dir}" \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX="${INSTALL}" \
          -DCMAKE_INSTALL_LIBDIR=lib \
          -DBUILD_TESTING=OFF \
          -DBUILD_TESTS=OFF \
          "$@"
    cmake --build "${build_dir}" -j "${JOBS}"
    cmake --install "${build_dir}"
}

build_ros() {     # $1..=包名
    echo "==> colcon build $*"
    ( cd "${HERE}" && \
      CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}" MAKEFLAGS="-j${JOBS}" \
      colcon build --merge-install --install-base "${INSTALL}" \
                   --build-base "${BUILD_ROOT}" \
                   --executor sequential \
                   --base-paths "${HERE}/rpg_vikit" "${HERE}/livox_ros_driver2" \
                                "${HERE}/fast_livo" "${HERE}/fast_lio" \
                   --packages-select "$@" \
                   --cmake-args -DCMAKE_BUILD_TYPE=Release \
                                -DBUILD_TESTING=OFF -DHUMBLE_ROS=humble -DROS_EDITION=ROS2 )
    source "${INSTALL}/local_setup.bash"
}

# 上游也由 build.sh 生成该清单；统一构建不能调用它清空 install。
if [ "${TARGET}" = all ] || [ "${TARGET}" = livox ]; then
    cp "${HERE}/livox_ros_driver2/package_ROS2.xml" "${HERE}/livox_ros_driver2/package.xml"
fi

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
    build_cmake "${HERE}/TEASER-plusplus" -DBUILD_TEASER_FPFH=ON -DBUILD_DOC=OFF "${TEASER_EXTRA[@]}"
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
if [ "${BUILD_FAST_LIO}" = 1 ] && \
   { [ "${TARGET}" = "all" ] || [ "${TARGET}" = "fast_lio" ]; }; then
    build_ros fast_lio
fi

echo
echo "完成。产物在 ${INSTALL}"
echo "之后 source \"${HERE}/../env.sh\"。"
