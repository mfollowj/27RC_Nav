#!/usr/bin/env bash
# ============================================================================
#  雷达链路一键体检：驱动起了吗？数据真的在流动吗？内容合理吗？
# ============================================================================
#  用法：
#      ./check_lidar.sh                 # 默认 /livox/lidar + /livox/imu
#      ./check_lidar.sh --secs 10       # 采样更久（默认 6 s）
#      LIDAR_TOPIC=/livox/lidar_filtered ./check_lidar.sh     # 换话题
#
#  它按 6 步查（每步 PASS/FAIL，最后给总判定与退出码 0/1）：
#      0 环境（ROS_DOMAIN_ID / GAZEBO_MASTER_URI / 节点数）
#      1 驱动节点与话题类型（有没有发布者）
#      2 数据流频率（雷达 / IMU 的 Hz）
#      3 消息内容（点数、z 范围、NaN 比例；IMU 的加速度模长、零偏）
#      4 时间戳是否在前进（防"只有一个缓存的旧消息"）
#      5 下游是否真的用上了（/cloud_registered、/Odometry、TF odom->base_link）
# ============================================================================
set +u
cd "$(dirname "$0")"

SECS=6
while [ $# -gt 0 ]; do
    case "$1" in
        --secs) SECS="$2"; shift 2;;
        -h|--help) sed -n '2,20p' "$0"; exit 0;;
        *) echo "未知参数 $1（见 --help）"; exit 2;;
    esac
done

LIDAR="${LIDAR_TOPIC:-/livox/lidar}"
IMU="${IMU_TOPIC:-/livox/imu}"

ok=0; bad=0
pass() { echo "  [PASS] $*"; ok=$((ok+1)); }
fail() { echo "  [FAIL] $*"; bad=$((bad+1)); }
info() { echo "         $*"; }

echo "=================================================================="
echo " 雷达链路体检  雷达=${LIDAR}  IMU=${IMU}  采样 ${SECS}s"
echo "=================================================================="

# ---------- 0 环境 ----------
echo "[0/5] 环境"
if [ -f ./env.sh ]; then . ./env.sh >/dev/null 2>&1 && info "已 source ./env.sh"; else info "没找到 env.sh（真机可能不需要）"; fi
info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<未设>}  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<未设>}"
if ! command -v ros2 >/dev/null 2>&1; then fail "找不到 ros2（先 source /opt/ros/humble/setup.bash）"; echo; echo "总判定：FAIL"; exit 1; fi
NODES=$(timeout 8 ros2 node list 2>/dev/null | wc -l)
[ "${NODES}" -gt 0 ] && pass "ROS 图里有 ${NODES} 个节点" || fail "ROS 图里一个节点都没有（仿真/驱动都没起？）"

DRVNODE=$(timeout 8 ros2 node list 2>/dev/null | grep -iE 'livox|driver|lidar' | head -3)
[ -n "${DRVNODE}" ] && pass "发现雷达相关节点：$(echo ${DRVNODE} | tr '\n' ' ')" \
                     || fail "没发现雷达驱动节点（真机: livox_ros_driver2；仿真: gzserver 的 livox 插件）"

if [ "${NODES}" -eq 0 ] || [ -z "${DRVNODE}" ]; then
  echo "[0.5/5] 域体检：当前域 ${ROS_DOMAIN_ID:-0} 看不到驱动，扫一下其它域和进程"
  FOUNDD=""
  for d in 0 1 2 3 42 43; do
      [ "${d}" = "${ROS_DOMAIN_ID:-0}" ] && continue
      n=$(ROS_DOMAIN_ID=${d} timeout 6 ros2 node list 2>/dev/null | wc -l)
      p=$(ROS_DOMAIN_ID=${d} timeout 6 ros2 topic info "${LIDAR}" 2>/dev/null | grep -oP 'Publisher count: \K\d+')
      if [ "${n:-0}" -gt 0 ] && [ "${p:-0}" -gt 0 ]; then
          info "域 ${d}：${n} 个节点，且有 ${LIDAR}（${p} 个发布者）  <== 驱动在这个域"
          FOUNDD="${d}"
      fi
  done
  PIDS=$(pgrep -f 'livox_ros_driver2_nod[e]|fastlivo_mappin[g]|fastlio_mappin[g]' 2>/dev/null)
  if [ -n "${PIDS}" ]; then
      info "本机确有雷达/LIO **进程**在跑（real_map.sh 就是按这个判的，与 ROS 域无关）："
      for pid in ${PIDS}; do
          dd=$(tr '\0' '\n' < /proc/${pid}/environ 2>/dev/null | grep '^ROS_DOMAIN_ID=' | cut -d= -f2)
          info "    pid ${pid}  ROS_DOMAIN_ID=${dd:-<未设=0>}"
      done
      if [ -n "${FOUNDD}" ]; then
          info "结论：驱动在域 ${FOUNDD}，本项目脚本用的是域 ${ROS_DOMAIN_ID:-0} —— 两边看不到彼此。"
          info "  ① 让驱动进本项目域： ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} <启动驱动的命令>"
          info "  ② 或把本项目脚本换到驱动的域： RC_DOMAIN_ID=${FOUNDD} ./real_map.sh"
      else
          info "结论：有驱动进程，但任何探测到的 ROS 域里都没有它的节点（正在初始化/已崩溃/被网络隔离）。"
          info "  等它起来，或先停掉： pkill -f livox_ros_driver2_node"
      fi
  else
      info "本机也没有雷达驱动进程 —— 那就是真的没起驱动。"
  fi
fi
# ---------- 1 话题与发布者 ----------
echo "[1/5] 话题与发布者"
check_topic() {
    local t="$1" name="$2"
    local inf; inf=$(timeout 8 ros2 topic info "$t" 2>/dev/null)
    if [ -z "${inf}" ]; then fail "${name} 话题 ${t} 不存在"; return 1; fi
    local pub; pub=$(echo "${inf}" | grep -oP 'Publisher count: \K\d+')
    local typ; typ=$(echo "${inf}" | grep -oP 'Type: \K.*')
    if [ "${pub:-0}" -ge 1 ]; then pass "${name} ${t} 有 ${pub} 个发布者（类型 ${typ}）"
    else fail "${name} ${t} 没有发布者（驱动没起或话题名不对）"; return 1; fi
}
check_topic "${LIDAR}" "雷达" ; LIDAR_OK=$?
check_topic "${IMU}"   "IMU " ; IMU_OK=$?

# ---------- 2 频率 ----------
echo "[2/5] 数据流频率（至少 1 Hz 才算在流）"
hz_of() { timeout $((SECS+3)) ros2 topic hz "$1" 2>/dev/null | grep -oP 'average rate: \K[\d.]+' | head -1; }
LIDAR_HZ=""; IMU_HZ=""
[ "${LIDAR_OK}" = 0 ] && { LIDAR_HZ=$(hz_of "${LIDAR}"); [ -n "${LIDAR_HZ}" ] && pass "雷达 ${LIDAR}: ${LIDAR_HZ} Hz" || fail "雷达 ${LIDAR}: 收不到消息（0 Hz）"; }
[ "${IMU_OK}" = 0 ]   && { IMU_HZ=$(hz_of "${IMU}");     [ -n "${IMU_HZ}" ]   && pass "IMU  ${IMU}: ${IMU_HZ} Hz"     || fail "IMU  ${IMU}: 收不到消息（0 Hz）"; }

# ---------- 3 消息内容（点数/z 范围/NaN、IMU 模长/零偏） ----------
echo "[3/5] 消息内容合理性"
CKOUT=$(python3 - "${LIDAR}" "${IMU}" "${SECS}" <<'PY'
import sys, math, time
import rclpy
from rclpy.node import Node
try:
    from rosidl_runtime_py.utilities import get_message
except Exception:
    get_message = None
lidar_t, imu_t, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
rclpy.init()
node = Node('lidar_check')
import numpy as np
res = {}
def sub(topic, tag):
    try:
        for _ in range(24):          # 等 ROS 图同步
            if any(x[0] == topic for x in node.get_topic_names_and_types()):
                break
            rclpy.spin_once(node, timeout_sec=0.25)
        ty = node.get_topic_names_and_types()
        t = [x for x in ty if x[0] == topic]
        if not t:
            print('RES FAIL %s 话题 %s 不存在' % (tag, topic)); return
        cls = get_message(t[0][1][0])
        box = {'n': 0, 'stamps': [], 'pts': [], 'zmin': None, 'zmax': None, 'zsum': 0.0, 'npts': 0,
               'nan': 0, 'acc': [], 'gyr': []}
        def cb(m):
            box['n'] += 1
            try: box['stamps'].append(float(m.header.stamp.sec) + 1e-9 * m.header.stamp.nanosec)
            except Exception: pass
            if hasattr(m, 'fields'):          # PointCloud2
                try:
                    a = np.frombuffer(bytes(m.data), dtype=np.float32)
                    k = len(m.fields)
                    if k >= 3 and a.size >= 3:
                        a = a[: (a.size // k) * k].reshape(-1, k)
                        z = a[:, 2]
                        box['npts'] += len(z); box['nan'] += int(np.isnan(z).sum())
                        box['zsum'] += float(np.nan_to_num(z).sum())
                        zz = z[np.isfinite(z)]
                        if zz.size:
                            box['zmin'] = float(zz.min()) if box['zmin'] is None else min(box['zmin'], float(zz.min()))
                            box['zmax'] = float(zz.max()) if box['zmax'] is None else max(box['zmax'], float(zz.max()))
                except Exception as e:
                    box.setdefault('err', str(e))
            elif hasattr(m, 'point_num'):     # Livox CustomMsg
                box['npts'] += int(m.point_num)
            if hasattr(m, 'linear_acceleration'):
                ax = m.linear_acceleration
                box['acc'].append(math.sqrt(ax.x**2 + ax.y**2 + ax.z**2))
                g = m.angular_velocity
                box['gyr'].append(math.sqrt(g.x**2 + g.y**2 + g.z**2))
        s = node.create_subscription(cls, topic, cb, 10)
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_subscription(s)
        res[topic] = box
    except Exception as e:
        print('RES FAIL %s 订阅 %s 失败: %s' % (tag, topic, e))
sub(lidar_t, '雷达')
sub(imu_t, 'IMU ')
for topic, b in res.items():
    if b['n'] == 0:
        print('RES FAIL %s：%ss 内一条消息都没收到' % (topic, secs)); continue
    adv = '未知'
    if len(b['stamps']) >= 2:
        adv = '在前进' if b['stamps'][-1] > b['stamps'][0] else '没前进(可疑)'
    if b['acc'] or b['gyr']:
        if b['acc']:
            acc = float(np.mean(b['acc'])); gyr = float(np.mean(b['gyr'])) if b['gyr'] else -1
            good = 8.5 < acc < 11.0
            print('  [%s] %s：%d 条 / %.0f Hz；加速度模长 %.2f m/s^2（静止应 ~9.8）%s；角速度模长 %.3f rad/s'
                  % ('PASS' if good else 'FAIL', topic, b['n'], b['n'] / secs, acc,
                     '' if good else ' <- 不像重力，检查 IMU 标定/坐标系', gyr))
        else:
            print('RES WARN %s：%d 条，但没解析出 IMU 字段' % (topic, b['n']))
    else:
        npts = b['npts']
        if npts == 0:
            print('RES FAIL %s：%d 帧但点数为 0（雷达空回波）' % (topic, b['n']))
        else:
            per = npts / max(1, b['n'])
            nanp = 100.0 * b['nan'] / max(1, npts)
            zr = ''
            if b['zmin'] is not None:
                zr = '；z 范围 %.2f~%.2f m' % (b['zmin'], b['zmax'])
            good = per >= 100 and nanp < 30
            print('  [%s] %s：%d 帧 / %.0f Hz；每帧 %.0f 点；NaN %.1f%%%s；时间戳%s'
                  % ('PASS' if good else 'FAIL', topic, b['n'], b['n'] / secs, per, nanp, zr, adv))
node.destroy_node()
rclpy.shutdown()
PY
)
echo "${CKOUT}" | grep -v '^RES ' || true
while read -r _t _v _rest; do
    [ "${_t}" = RES ] || continue
    case "${_v}" in
        PASS) pass "${_rest}";;
        WARN) info "[WARN] ${_rest}";;
        *)    fail "${_rest}";;
    esac
done < <(echo "${CKOUT}" | grep '^RES ')

# ---------- 4 下游：LIO 有没有真的用上 ----------
echo "[4/5] 下游（LIO 是否真的用上了这份数据）"
for t in /cloud_registered /Odometry; do
    h=$(timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -oP 'average rate: \K[\d.]+' | head -1)
    if [ -n "${h}" ]; then pass "${t}: ${h} Hz"; else fail "${t} 没有数据（LIO 没起来/没收到雷达）"; fi
done
if timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q Translation; then
    pass "TF odom->base_link 存在"
else
    fail "没有 TF odom->base_link（LIO 未发布位姿）"
fi

# ---------- 5 汇总 ----------
# 第3步的结果来自上面的 python 子进程，这里把它也计入总判定
ok=$((ok + $(echo "${CKOUT}" | grep -c "\[PASS\]")))
bad=$((bad + $(echo "${CKOUT}" | grep -c "\[FAIL\]")))
echo "[5/5] 汇总"
echo "------------------------------------------------------------------"
if [ "${bad}" = 0 ] && [ -n "${LIDAR_HZ}" ]; then
    echo "总判定：PASS（${ok} 项通过）—— 雷达驱动与数据链路正常"
    exit 0
else
    echo "总判定：FAIL（${ok} 项通过，${bad} 项失败）"
    echo "排查顺序：1) 驱动是否启动  2) 话题名/类型是否对  3) 网线/仿真是否在跑"
    echo "            4) ROS_DOMAIN_ID 是否与驱动一致（本仓库默认 42）"
    exit 1
fi
