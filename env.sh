#!/usr/bin/env bash
# ============================================================================
#  SCU27RC_Nav 统一环境（所有一键脚本都会 source 它，手敲 ros2 命令前也建议 source）
#      source /home/zx/ITEM/SCU27RC_Nav/env.sh
# ============================================================================
#  ★ 私有 DDS 域 / Gazebo 端口
#    ROS 2 默认 ROS_DOMAIN_ID=0 是"全网共用"的，同机还有别的 ROS 2 仿真在发
#    /clock、/livox/*、/tf、/cmd_vel，会互相污染（LIO 收到别人的 LiDAR/IMU、
#    Nav2 串车、RViz 丢消息）。所以本项目固定用私有域（默认 42）。
export ROS_DOMAIN_ID="${RC_DOMAIN_ID:-42}"

#    Gazebo Classic 默认 11345 也是全机共用，本项目用 11355。
export GAZEBO_MASTER_URI="${RC_GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"

# 只想本机通信、连 UDP 多播都不发（远程机器上看不到本项目话题，按需打开）
# export ROS_LOCALHOST_ONLY=1

# ---------------------------------------------------------------------------
_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RC_ROOT="${_root}"
RC_DEPS_INSTALL="${RC_DEPS_INSTALL:-${_root}/Deps/install}"
RC_WS_INSTALL="${_root}/Nav/ros2_ws/install"

# ---------------------------------------------------------------------------
#  0) 清洗继承来的环境：终端里可能还留着别的工作区（例如旧 SCURC_Nav_Sim、
#     getting_relocal），它们会**排在前面**把同名包（fast_livo / livox /
#     nav2_bringup）抢走 —— 本项目的自包含就名存实亡。
#     这里把这些前缀从环境里摘掉，再由下面按固定顺序重新 source。
# ---------------------------------------------------------------------------
_rc_stale_prefixes() {
    printf '%s\n' \
        /home/zx/SCURC_Nav_Sim \
        /home/zx/getting_relocal \
        /home/rc/New_Nav \
        /home/rc/SCURM_SentryNavigation \
        "${RC_DEPS_INSTALL}" \
        "${RC_WS_INSTALL}" \
        "${_root}"
}

_rc_drop_prefixes() {   # $1=环境变量名（冒号分隔），其余=要摘掉的前缀
    local var="$1"; shift
    local out="" entry p bad
    local -a parts
    IFS=':' read -ra parts <<< "${!var:-}"
    for entry in "${parts[@]}"; do
        [ -z "${entry}" ] && continue
        bad=0
        for p in "$@"; do
            case "${entry}" in
                "${p}"|"${p}"/*) bad=1; break ;;
            esac
        done
        [ "${bad}" = "1" ] || out="${out:+${out}:}${entry}"
    done
    export "${var}=${out}"
}

mapfile -t _rc_stale < <(_rc_stale_prefixes)
for _v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
          LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH GAZEBO_MODEL_PATH PATH; do
    [ -n "${!_v:-}" ] && _rc_drop_prefixes "${_v}" "${_rc_stale[@]}"
done
unset _v

source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo-11/setup.sh ] && source /usr/share/gazebo-11/setup.sh >/dev/null 2>&1

# 1) 本项目自带的非官方依赖（Deps/，见 Deps/README.md 与 Deps/build_deps.sh）
#    ★ 现在是**唯一**来源：fast_livo / fast_lio / livox_ros_driver2 / vikit_* /
#      small_gicp / TEASER++ 全部装在这里，不再依赖任何外部工作区。
if [ -f "${RC_DEPS_INSTALL}/setup.bash" ]; then
    source "${RC_DEPS_INSTALL}/setup.bash" >/dev/null 2>&1
else
    echo "[env] 警告：${RC_DEPS_INSTALL}/setup.bash 不存在。" >&2
    echo "[env]       先编译依赖：./Deps/build_deps.sh" >&2
fi

# 1.1) TEASER++ / small_gicp 是普通 CMake 安装（不是 ament 包），ld 不会自动找到。
#      这里统一补 LD_LIBRARY_PATH，真机/仿真两条链路就不用各自再拼一把。
if [ -d "${RC_DEPS_INSTALL}/lib" ]; then
    case ":${LD_LIBRARY_PATH:-}:" in
        *":${RC_DEPS_INSTALL}/lib:"*) ;;
        *) export LD_LIBRARY_PATH="${RC_DEPS_INSTALL}/lib:${LD_LIBRARY_PATH:-}" ;;
    esac
fi

# 2) 本项目 ROS 工作区（rc_field_nav + relocalization + 仿真雷达插件）
[ -f "${RC_WS_INSTALL}/setup.bash" ] && source "${RC_WS_INSTALL}/setup.bash" >/dev/null 2>&1

# 无 X 显示（SSH/远程）默认不开 RViz：rviz2 在无显示时会因 XCB 直接崩
if [ -z "${RC_RVIZ:-}" ]; then
    if [ -n "${DISPLAY:-}" ]; then export RC_RVIZ=true; else export RC_RVIZ=false; fi
fi
# 只有本机确实有 X0 时才兜底 DISPLAY，避免在远程把 DISPLAY 设成 :0 导致 rviz 必崩
[ -S /tmp/.X11-unix/X0 ] && export DISPLAY="${DISPLAY:-:0}"
unset _root RC_DEPS_INSTALL RC_WS_INSTALL
unset -f _rc_stale_prefixes _rc_drop_prefixes
