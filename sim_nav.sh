#!/usr/bin/env bash
# ③ 仿真导航：先验地图 + 导航中持续重定位（不用 AMCL）+ Nav2
#    前提：已经用 ./sim_save_map.sh 存过图，且**出生点与建图时一致**
#
#    ./sim_nav.sh                                     # ★ 最常用：自动用最近一次存图
#    ./sim_nav.sh prior_map:=<绝对路径> map:=<绝对路径>   # 指定地图（要成对、同一次存图）
#    ./sim_nav.sh params_file:=<绝对路径>              # 换代价地图参数
#    发目标点（或 RViz 里 2D Goal Pose）：
#      ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
#        "{pose: {header: {frame_id: map}, pose: {position: {x: 3.9, y: 4.5}, orientation: {w: 1.0}}}}"
set +u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 私有 DDS 域(42) + 私有 Gazebo 端口(11355) + 全部环境依赖，说明见 env.sh
source "${ROOT}/env.sh"
echo "[环境] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"

MAPS="${ROOT}/Nav/ros2_ws/install/rc_field_nav/share/rc_field_nav/maps"

# ---------------------------------------------------------------------------
# 解析用户显式给的地图参数（形如 name:=value；路径里的引号 bash 已经去掉）
# ---------------------------------------------------------------------------
prior_arg=""; map_arg=""; params_arg=""
for a in "$@"; do
    case "$a" in
        prior_map:=*)   prior_arg="${a#prior_map:=}" ;;
        map:=*)         map_arg="${a#map:=}" ;;
        params_file:=*) params_arg="${a#params_file:=}" ;;
    esac
done

# ---------------------------------------------------------------------------
# 启动前校验：路径写错时当场给出可照抄的修正命令
#   为什么放在脚本里：定位器是被 launch 拉起来的，它的 print 不会进 launch.log，
#   事后只能看到一行 "process has died"（实测踩过：用户把示例里的 /abs/path/ 原样粘了）
# ---------------------------------------------------------------------------
bad=0
check() {   # $1=标签  $2=路径
    [ -z "$2" ] && return 0
    if [ ! -s "$2" ]; then
        echo "[错误] $1 不存在或为空：$2"
        bad=1
    fi
}
check "先验 3D 点云（prior_map）" "$prior_arg"
check "先验 2D 代价地图（map）" "$map_arg"
check "Nav2 参数文件（params_file）" "$params_arg"

if [ "$bad" = "1" ]; then
    echo
    echo "【maps 目录里现有的文件】${MAPS}"
    ls -1 "${MAPS}" 2>/dev/null | grep -v gitkeep | sed "s/^/    /"
    echo
    echo "【最简单的用法】不传地图参数，脚本会自动用最近一次存图："
    echo "    ./sim_nav.sh"
    echo "【或照抄】把 <名字> 换成上面列出的名字："
    echo "    ./sim_nav.sh prior_map:=${MAPS}/<名字>_map.pcd map:=${MAPS}/<名字>_costmap.yaml"
    case "${prior_arg}${map_arg}" in
        */Nav/ros2_ws/*|*/Nav/real/*|*/Nav/sim/*)
            echo
            echo "提示：你给的路径少写了项目前缀 —— 本机完整路径必须以 ${ROOT} 开头。"
            ;;
        /abs/*)
            echo
            echo "提示：/abs/path/... 是文档里的**占位示例**，不是真实路径，请换成上面列出的真实文件。"
            ;;
    esac
    exit 1
fi

# ---------------------------------------------------------------------------
# 组装最终参数：显式给的优先；只给一个时按同名配对自动补另一半；都没给则自动挑
# ---------------------------------------------------------------------------
EXTRA=()
if [ -n "$prior_arg" ] || [ -n "$map_arg" ]; then
    if [ -n "$prior_arg" ] && [ -z "$map_arg" ]; then
        cand="${prior_arg%_map.pcd}_costmap.yaml"
        if [ -s "$cand" ]; then
            echo "[提示] 只给了 prior_map，按同名配对自动补上 map:=$cand"
            map_arg="$cand"
        else
            echo "[警告] 只给了 prior_map，没给成对的 map（2D 代价地图）——默认会去用"
            echo "       ${MAPS}/prior_costmap.yaml，可能和你的 3D 点云不是同一次存图。"
        fi
    fi
    if [ -n "$map_arg" ] && [ -z "$prior_arg" ]; then
        cand="${map_arg%_costmap.yaml}_map.pcd"
        if [ -s "$cand" ]; then
            echo "[提示] 只给了 map，按同名配对自动补上 prior_map:=$cand"
            prior_arg="$cand"
        else
            echo "[警告] 只给了 map，没给成对的 prior_map（3D 点云）——重定位会用旧点云。"
        fi
    fi
    [ -n "$prior_arg" ] && EXTRA+=(prior_map:="$prior_arg")
    [ -n "$map_arg" ] && EXTRA+=(map:="$map_arg")
else
    if [ -s "${MAPS}/prior_map.pcd" ] && [ -s "${MAPS}/prior_costmap.yaml" ]; then
        EXTRA=()
    else
        PAIR=""
        for y in $(ls -t "${MAPS}"/*_costmap.yaml 2>/dev/null); do
            n="$(basename "${y}" _costmap.yaml)"
            if [ -s "${MAPS}/${n}_map.pcd" ]; then PAIR="${n}"; break; fi
        done
        if [ -n "${PAIR}" ]; then
            echo "[提示] 没有默认的 prior_map.pcd，自动使用最近一次存图：${PAIR}_map.pcd + ${PAIR}_costmap.yaml"
            EXTRA=(prior_map:="${MAPS}/${PAIR}_map.pcd" map:="${MAPS}/${PAIR}_costmap.yaml")
        else
            echo "[错误] 还没有先验地图。先跑 ./sim_map.sh 建图 + ./sim_save_map.sh 存图。"
            exit 1
        fi
    fi
fi

# 只看参数判断结果、不真的启动（排查地图路径时很有用）
if [ "${1:-}" = "--dry-run" ]; then
    echo "[dry-run] 最终会传给 launch 的地图参数："
    if [ ${#EXTRA[@]} -eq 0 ]; then
        echo "    (没有，使用 launch 默认值：maps/prior_map.pcd + maps/prior_costmap.yaml)"
    else
        for e in "${EXTRA[@]}"; do echo "    $e"; done
    fi
    exit 0
fi

echo "[1/2] 清理上一次残留的仿真进程"
ros2 run rc_field_nav kill_sim.sh >/dev/null 2>&1 || true
echo "[2/2] 启动先验图导航（重定位 + Nav2）"
exec ros2 launch rc_field_nav nav_prior_map.launch.py gui:="${RC_GUI:-true}" rviz:="${RC_RVIZ:-true}" "${EXTRA[@]}" "$@"
