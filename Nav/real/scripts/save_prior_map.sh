#!/usr/bin/env bash
# ============================================================================
#  真机·存图：把 real_map.sh 攒在内存里的建图结果落成"先验地图三件套"
# ============================================================================
#  前提：./real_map.sh 还在跑（累积器把 /cloud_registered 攒在内存里），
#        并且车已经开着走过场地。Ctrl-C 掉 real_map.sh 之后图就没了。
#
#  用法：
#      ./real_save_map.sh --imu-height 0.30                      # 从内存累积器存
#      ./real_save_map.sh --cloud real/maps/prior_map.pcd \
#                         --traj  real/maps/prior_traj.txt         # 用已有文件离线重跑
#
#  产物（real/maps/）：
#      prior_map.pcd            先验 3D 点云（odom 系，稠密）
#      prior_traj.txt           建图轨迹
#      prior_costmap.pgm/.yaml  Nav2 map_server 直接能用的 2D 占据栅格
#      prior_costmap.meta.json  ground_z / imu_height / 分辨率 + terrain 统计
#      prior_costmap_elev.npy   当地地面高度场（方案A 中间产物）
#
#  ★ 为什么不再抓 /Laser_map：
#    FAST-LIVO2 只在 locate_in_prior_map=true（先验图跟踪）模式启动那一刻，
#    把先验图往 /Laser_map 发一次；建图模式（本阶段）**根本不发布这个话题**。
#    真机的点云来自 real_mapping.launch.py 里的内存累积器：
#        save_cloud_pcd.py --topic /cloud_registered --accumulate \
#                          --trigger-topic /save_prior_map -o real/maps/prior_map.pcd
#    所以这里要做的是"发一条 /save_prior_map 让累积器写盘"，而不是订阅 /Laser_map。
# ============================================================================
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${ROOT}/env.sh"

NAME="prior"
if [ $# -gt 0 ] && [ "${1#-}" = "${1}" ]; then
    NAME="$1"; shift
fi
EXTRA_ARGS=("$@")

# ---- 把 --cloud / --traj 摘出来自己处理；其余原样透传给 map3d_to_costmap.py ----
CLOUD_IN=""
TRAJ_IN=""
_F=()
_i=0
while [ "${_i}" -lt "${#EXTRA_ARGS[@]}" ]; do
    case "${EXTRA_ARGS[$_i]}" in
        --cloud)        CLOUD_IN="${EXTRA_ARGS[$((_i+1))]}"; _i=$((_i+2));;
        --cloud=*)      CLOUD_IN="${EXTRA_ARGS[$_i]#*=}";     _i=$((_i+1));;
        --traj)         TRAJ_IN="${EXTRA_ARGS[$((_i+1))]}";   _i=$((_i+2));;
        --traj=*)       TRAJ_IN="${EXTRA_ARGS[$_i]#*=}";      _i=$((_i+1));;
        *)              _F+=("${EXTRA_ARGS[$_i]}");           _i=$((_i+1));;
    esac
done
EXTRA_ARGS=("${_F[@]}")

# ---- 有效的 IMU 离地高度（默认值 + 用户在 EXTRA_ARGS 里的最后一次覆盖）----
IMU_H="${RC_IMU_HEIGHT:-0.30}"
_j=0
while [ "${_j}" -lt "${#EXTRA_ARGS[@]}" ]; do
    case "${EXTRA_ARGS[$_j]}" in
        --imu-height)   IMU_H="${EXTRA_ARGS[$((_j+1))]}";;
        --imu-height=*) IMU_H="${EXTRA_ARGS[$_j]#*=}";;
    esac
    _j=$((_j+1))
done

# 脚本优先用**源码树**里的（不依赖 colcon install；换机器/没编译也能存图）
LIB="${ROOT}/Nav/ros2_ws/src/rc_field_nav/scripts"
if [ ! -f "${LIB}/map3d_to_costmap.py" ]; then
    PKG_PREFIX="$(ros2 pkg prefix rc_field_nav 2>/dev/null || true)"
    [ -n "${PKG_PREFIX}" ] && LIB="${PKG_PREFIX}/lib/rc_field_nav"
fi
MAPS="${RC_MAPS_DIR:-${ROOT}/Nav/real/maps}"
ACCUM="${MAPS}/prior_map.pcd"        # real_mapping.launch.py 的 map_out 默认值
TRAJ_DEF="${MAPS}/prior_traj.txt"    # real_mapping.launch.py 的 traj_out 默认值
mkdir -p "${MAPS}"

echo "=============================================================="
echo "建图产物目录：${MAPS}   （IMU 离地高度 ${IMU_H} m）"

# ---------------------------------------------------------------------------
# 1) 点云
# ---------------------------------------------------------------------------
CLOUD_USED=""
if [ -n "${CLOUD_IN}" ]; then
    echo "[1/4] 离线模式：直接使用 ${CLOUD_IN}"
    [ -s "${CLOUD_IN}" ] || { echo "[错误] 点云文件不存在或为空：${CLOUD_IN}"; exit 1; }
    CLOUD_USED="${CLOUD_IN}"
else
    echo "[1/4] 触发内存累积器落盘（/save_prior_map）"
    if ! ros2 topic list 2>/dev/null | grep -qx '/cloud_registered'; then
        echo "      [警告] ROS 图（域 ${ROS_DOMAIN_ID}）里没有 /cloud_registered ——"
        echo "             ./real_map.sh 大概没在跑，或雷达/LIO 没起来："
        echo "             ros2 topic hz /livox/lidar_filtered   ros2 topic hz /cloud_registered"
    fi
    _t0=$(date +%s)
    # --once 默认会等 1 个订阅者；套 timeout 防止累积器没起来时卡死
    timeout 8 ros2 topic pub --once /save_prior_map std_msgs/msg/Empty {} >/dev/null 2>&1 \
        || echo "      [警告] /save_prior_map 没有被接住（累积器没在跑？map_accum:=false？）"

    CLOUD_SRC=""
    for _ in $(seq 1 30); do
        if [ -s "${ACCUM}" ] && [ "$(stat -c %Y "${ACCUM}")" -ge "${_t0}" ]; then
            CLOUD_SRC="${ACCUM}"; break
        fi
        sleep 1
    done

    if [ -n "${CLOUD_SRC}" ]; then
        echo "      累积器已写盘 -> ${ACCUM} ($(du -h "${ACCUM}" | cut -f1))"
    else
        echo "      [警告] 30 s 内没等到累积器写盘，退回抓实时地图快照。"
        echo "             快照只有当前视野 / 或降采样后的图，墙会变虚线。"
        for _t in /prior_map_accum /Laser_map; do
            if python3 "${LIB}/save_pointcloud_pcd.py" -t "${_t}" \
                    -o "${MAPS}/${NAME}_map.pcd" --timeout 15; then
                CLOUD_SRC="${MAPS}/${NAME}_map.pcd"; break
            fi
        done
        [ -n "${CLOUD_SRC}" ] || { echo "[错误] 没拿到任何点云，放弃。"; exit 1; }
    fi

    CLOUD_DST="${MAPS}/${NAME}_map.pcd"
    [ "${CLOUD_SRC}" = "${CLOUD_DST}" ] || cp -f "${CLOUD_SRC}" "${CLOUD_DST}"
    CLOUD_USED="${CLOUD_DST}"
fi

# ---------------------------------------------------------------------------
# 2) 轨迹
# ---------------------------------------------------------------------------
TRAJ_USED=""
echo "[2/4] 轨迹 -> ${MAPS}/${NAME}_traj.txt"
if [ -n "${TRAJ_IN}" ]; then
    [ -s "${TRAJ_IN}" ] || { echo "[错误] 轨迹文件不存在或为空：${TRAJ_IN}"; exit 1; }
    TRAJ_USED="${TRAJ_IN}"
elif [ -s "${TRAJ_DEF}" ]; then
    TRAJ_USED="${MAPS}/${NAME}_traj.txt"
    [ "${TRAJ_DEF}" = "${TRAJ_USED}" ] || cp -f "${TRAJ_DEF}" "${TRAJ_USED}"
else
    echo "      [警告] 找不到 ${TRAJ_DEF}（real_map.sh 没记录轨迹？record_traj:=false？）"
    echo "             没有轨迹就只能靠点云 z 的低分位数估地面。"
fi
[ -n "${TRAJ_USED}" ] && echo "      $(wc -l < "${TRAJ_USED}") 行"

# 单位自检：IMU 离地高度是"米"
if awk "BEGIN{exit !(${IMU_H} < 0.05 || ${IMU_H} > 2.0)}" 2>/dev/null; then
    echo "      [警告] --imu-height ${IMU_H} 看起来不是米：IMU 离地一般 0.15~0.60；"
    echo "             量到 30 cm 就写 0.30（不是 0.0030，也不是 30）。"
fi

# ---------------------------------------------------------------------------
# 3) 3D -> 2D
# ---------------------------------------------------------------------------
echo "[3/4] 3D 点云 -> 2D 代价地图（+ meta.json）"
ARGS=(--cloud "${CLOUD_USED}" -o "${MAPS}/${NAME}_costmap"
      --imu-height "${RC_IMU_HEIGHT:-0.30}" --h-min 0.15 --h-max 2.0 --resolution 0.05)
[ -n "${TRAJ_USED}" ] && ARGS+=(--trajectory "${TRAJ_USED}")
# 用户透传的额外参数放最后，可以覆盖上面的默认值（后出现的同名参数生效）
ARGS+=("${EXTRA_ARGS[@]}")
python3 "${LIB}/map3d_to_costmap.py" "${ARGS[@]}" || exit 1

python3 "${LIB}/map_sanity_check.py" "${MAPS}/${NAME}_costmap.yaml" || true

# ---------------------------------------------------------------------------
# 4) 先验图体检：这张 3D 图能不能拿去重定位
# ---------------------------------------------------------------------------
echo "[4/4] 先验图体检（重定位可用性）"
MAP_CHECK_ARGS=(--pcd "${CLOUD_USED}" --meta "${MAPS}/${NAME}_costmap.meta.json")
[ -n "${TRAJ_USED}" ] && MAP_CHECK_ARGS+=(--traj "${TRAJ_USED}")
if ! python3 "${ROOT}/Nav/real/scripts/check_prior_map.py" "${MAP_CHECK_ARGS[@]}"; then
    echo "      [警告] 体检不合格：这张图拿去重定位大概率失败。"
    echo "             常见原因：原地甩雷达（旋转拖影）、车没走遍场地、--imu-height 单位写错。"
    echo "             重做：./real_map.sh（雷达固定、慢速走遍场地）→ ./real_save_map.sh --imu-height 0.30"
fi

echo "--------------------------------------------------------------"
ls -la "${MAPS}" | grep "${NAME}" || true
echo "下一步（导航）："
echo "  ./real_nav.sh                        # 6-DOF 重定位 + Nav2（默认）"
echo "  ./real_reloc.sh rviz:=true           # 只调定位，不起 Nav2"
echo "  ./real_nav.sh reloc:=false imu_height:=${IMU_H} initial_pose:=\"x,y,yaw_deg\"   # 旧 4-DOF"
echo "  （先验 3D 图与 2D 图必须来自同一次存图）"
