#!/usr/bin/env bash
# 真机·建图：Livox Mid-360 驱动 + 抽稀 + FAST-LIVO2(无视觉) + 全程地图累积器 + RViz
#    走完场地后另开终端：./real_save_map.sh
#
#    ./real_map.sh                         # 起驱动+建图（GUI 用 RViz）
#    ./real_map.sh rviz:=false             # 不开 RViz
#    ./real_map.sh livox_driver:=false     # 驱动已在别的终端跑（前端复用）
#
#  ★ 上车前先确认雷达有数据：ros2 topic hz /livox/lidar（应 15 Hz）、/livox/imu（应 200 Hz）
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 私有 DDS 域(42) + 私有 Gazebo 端口(11355) + 全部环境依赖，说明见 env.sh
source "${ROOT}/env.sh"
echo "[环境] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
ROOT_REAL="${ROOT}/Nav/real"

# 真机上的"只能有一套"自检：两个驱动同时连一台雷达会把时间戳搞乱
# （实测：LiDAR 时间戳倒退、LIO 丢弃全部 IMU、位姿乱跳）
#  ★ 这个检查按**进程名**判，与 ROS_DOMAIN_ID 无关；而 ./check_lidar.sh 是按 **ROS 图** 判
#    （分域）。两者结论不一致，通常就是"驱动在别的域里跑"：
#    域 42（本项目）ROS 图空着  <->  这里说进程已经在跑。
_rc_force=0
[ "${RC_FORCE:-0}" = "1" ] && _rc_force=1
_rc_args=()
for _a in "$@"; do
    if [ "${_a}" = "--force" ]; then _rc_force=1; else _rc_args+=("${_a}"); fi
done
set -- "${_rc_args[@]}"
PIDS=$(pgrep -f 'livox_ros_driver2_nod[e]|fastlivo_mappin[g]|fastlio_mappin[g]' 2>/dev/null)
if [ -n "${PIDS}" ]; then
  echo "[注意] 本机已有雷达驱动/LIO 进程在跑（按进程名判，与 ROS 域无关）："
  pgrep -af 'livox_ros_driver2_node|fastlivo_mapping|fastlio_mapping|livox_preprocess' | sed 's/^/    /'
  for pid in ${PIDS}; do
    d=$(tr '\0' '\n' < /proc/${pid}/environ 2>/dev/null | grep '^ROS_DOMAIN_ID=' | cut -d= -f2)
    echo "    -> pid ${pid} 的 ROS_DOMAIN_ID=${d:-<未设=0>}（本项目用 ${ROS_DOMAIN_ID}）"
  done
  echo "  两套驱动的风险：① 同域会互相污染；② 同一台雷达被两个进程抢 UDP 口。"
  echo "  若它是你自己另起的、或不在本项目域里、确认不冲突，可强制继续："
  echo "      RC_FORCE=1 ./real_map.sh        # 或 ./real_map.sh --force"
  echo "  先想确认它到底在哪个域、有没有数据： ./check_lidar.sh"
  if [ "${_rc_force}" != "1" ]; then echo "[中止] 未强制继续。"; exit 1; fi
  echo "  [RC_FORCE] 已忽略上面的进程，继续启动。"
fi

echo "启动真机建图（另开终端：./real_save_map.sh 存图）"
exec ros2 launch "${ROOT_REAL}/launch/real_mapping.launch.py" \
     livox_driver:="${RC_DRIVER:-true}" rviz:="${RC_RVIZ:-true}" "$@"
