#!/usr/bin/env bash
# ============================================================================
#  真机·3D 全局重定位链路（大范围漂移后可恢复）
# ============================================================================
#  前提：先验 3D 点云 real/maps/prior_map.pcd（由 ./real_map.sh + ./real_save_map.sh 产出）
#
#  用法：
#      ./real_reloc.sh                       # 雷达 + LIO + TEASER/GICP + bridge
#      ./real_reloc.sh rviz:=true            # 顺带开 RViz（Fixed Frame = map）
#      ./real_reloc.sh livox_driver:=false   # 驱动已在别的终端跑
#      ./real_reloc.sh nav2:=true            # 需要导航时（还要 2D 图，本阶段默认关）
#      ./real_reloc.sh map_path:=/abs/x.pcd  # 换先验图
#
#  与 ./real_nav.sh 的区别见 real/launch/real_reloc.launch.py 顶部说明。
#  日志里判断是否工作：
#      fastlivo_mapping: "Prior map seeded OK" / "★ 重定位恢复：odom 归零..."
#      reloc_bridge:     "★ 接受重定位 [FIRST]/[RECOVER ...]"
#      ros2 topic echo /reloc_status --once
# ============================================================================
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 私有 DDS 域(42) + 私有 Gazebo 端口(11355) + 全部环境依赖，说明见 env.sh
source "${ROOT}/env.sh"

# TEASER++ / small_gicp 的动态库不在 ROS 环境里，补进 LD_LIBRARY_PATH。
# 只认本仓库 Deps/install（env.sh 已自动加过一遍，这里保留是为了不经 env.sh 时也能跑）。
RC_DEPS_INSTALL="${RC_DEPS_INSTALL:-${ROOT}/Deps/install}"
for d in "${RC_DEPS_INSTALL}/lib" "${RC_DEPS_INSTALL}/teaserpp/lib" \
         "${RC_DEPS_INSTALL}/small_gicp/lib"; do
    [ -d "${d}" ] && export LD_LIBRARY_PATH="${d}:${LD_LIBRARY_PATH:-}"
done

echo "[环境] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"

# 只能有一套驱动：两个驱动抢一台雷达会把时间戳搞乱
if pgrep -f 'livox_ros_driver2_nod[e]' >/dev/null 2>&1; then
    echo "[注意] 已检测到 livox 驱动进程（按进程名判，与 ROS 域无关）："
    pgrep -af 'livox_ros_driver2_node' | sed 's/^/    /'
    echo "  如果那是你自己另起的，请加 livox_driver:=false，否则会有两套驱动抢雷达。"
fi

exec ros2 launch "${ROOT}/Nav/real/launch/real_reloc.launch.py" "$@"
