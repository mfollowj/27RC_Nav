#!/usr/bin/env bash
# ============================================================================
#  真机·导航：抽稀 + FAST-LIVO2 + 6-DOF 全局重定位（默认）+ Nav2
# ============================================================================
#  前提：./real_save_map.sh 已产出 robot/maps/prior_map.pcd + prior_costmap.yaml
#
#  用法：
#      ./real_nav.sh                          # 6-DOF 重定位（TEASER/GICP）+ Nav2
#      ./real_nav.sh rviz:=true
#      ./real_nav.sh nav2:=false rviz:=true   # 只看定位，不起 Nav2
#      ./real_nav.sh livox_driver:=false      # 驱动已在别的终端跑
#      ./real_nav.sh reloc:=false             # ★ 退回旧链路（4-DOF 体素重叠）
#
#  6-DOF 链路（默认）：
#      原始雷达 -> teaser_gicp_node -> /icp_result (map<-livox)
#               -> reloc_bridge     -> /relocalization_result (map<-base) + TF map->odom
#               -> fastlivo_mapping -> 收到稳定新位姿就重置 odom + 重建体素地图
#      所以大范围漂移（甩发散/被搬动）之后能自己找回来。
#      该链路**不需要** imu_height / initial_pose（全局搜索，无初值），传了会被忽略并提示。
#
#  旧链路（reloc:=false）：
#      prior_map_localizer.py 做 4-DOF（x/y/yaw）体素重叠，只修 map->odom，
#      LIO 发散后救不回来；需要 imu_height:= / initial_pose:= 时的行为与以前一致。
# ============================================================================
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 私有 DDS 域(42) + 私有 Gazebo 端口(11355) + 全部环境依赖，说明见 env.sh
source "${ROOT}/env.sh" || exit
echo "[环境] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
ROOT_REAL="${ROOT}/robot"

# 真机上的"只能有一套"自检：两个驱动同时连一台雷达会把时间戳搞乱
# （实测：LiDAR 时间戳倒退、LIO 丢弃全部 IMU、位姿乱跳）
START_DRIVER="${RC_DRIVER:-true}"
for a in "$@"; do
  case "$a" in livox_driver:=*) START_DRIVER="${a#livox_driver:=}" ;; esac
done
if { [ "${START_DRIVER}" != false ] && [ "${START_DRIVER}" != 0 ] && \
     pgrep -f 'livox_ros_driver2_nod[e]' >/dev/null 2>&1; } || \
   pgrep -f 'fastlivo_mappin[g]' >/dev/null 2>&1 || \
   pgrep -f 'fastlio_mappin[g]' >/dev/null 2>&1; then
  echo "[中止] 已经有雷达驱动/LIO 在跑。先全部停掉再启动，否则会同时存在两套："
  pgrep -af 'livox_ros_driver2_node|fastlivo_mapping|fastlio_mapping|livox_preprocess' | sed 's/^/    /'
  echo "  停掉： pkill -f livox_ros_driver2_node; pkill -f fastlivo_mapping; pkill -f livox_preprocess"
  exit 1
fi

# ---- 选择链路（先扫一遍 reloc:=，再分拣参数）----
RELOC=1
for a in "$@"; do
  case "$a" in
    reloc:=*) RELOC="${a#reloc:=}" ;;
  esac
done
case "${RELOC}" in
  1|true|True|TRUE) RELOC=1 ;;
  0|false|False|FALSE) RELOC=0 ;;
  *) echo "[错误] reloc 只能为 true/false 或 1/0：${RELOC}" >&2; exit 2 ;;
esac

if [ "${RELOC}" = "1" ]; then
  ARGS=()
  IGNORED=()
  for a in "$@"; do
    case "$a" in
      reloc:=*) ;;
      imu_height:=*|initial_pose:=*|min_score:=*|reloc_period:=*|front_end:=*)
        IGNORED+=("${a}") ;;
      *) ARGS+=("${a}") ;;
    esac
  done
  if [ ${#IGNORED[@]} -gt 0 ]; then
    echo "[提示] 6-DOF 链路用全局搜索，不需要这些参数，已忽略：${IGNORED[*]}"
  fi
  echo "启动真机 6-DOF 重定位 + 导航（RViz: real_reloc.rviz，Fixed Frame=map）"
  exec ros2 launch "${ROOT_REAL}/launch/real_relocalization.launch.py" \
       livox_driver:="${RC_DRIVER:-true}" rviz:="${RC_RVIZ:-true}" \
       nav2:=true "${ARGS[@]}"
else
  ARGS=()
  for a in "$@"; do
    case "$a" in reloc:=*) ;; *) ARGS+=("${a}") ;; esac
  done
  echo "启动真机 4-DOF 定位 + 导航（旧链路，LIO 发散后无法恢复）"
  exec ros2 launch "${ROOT_REAL}/launch/real_voxel_localization.launch.py" \
       livox_driver:="${RC_DRIVER:-true}" rviz:="${RC_RVIZ:-true}" "${ARGS[@]}"
fi
