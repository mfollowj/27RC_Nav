# mcu_bridge —— RViz 目标点 + 当前位姿 → 单片机串口

把**你在 RViz 里点出的目标点**（`/goal_pose`）和**机器人当前位姿**（优先 LIO 里程计，TF 作为后备），
按固定 15 字节帧通过 USB 转串口发给单片机。跑在机器人小电脑上；
本机（开发机）只跑 RViz，不碰串口。

```
RViz(本机) --/goal_pose--> mcu_bridge(机器人) --串口 0x7E…0x21--> 单片机
LIO --/aft_mapped_to_init；TF 作为后备--> ┘
```

## 线协议

两帧形状一样，靠帧头后面的 **ID 字节**区分：

| 偏移 | 字段 | 类型 | 值 / 含义 |
|---:|---|---|---|
| 0 | SOF | uint8 | `0x7E` |
| 1 | ID | uint8 | `0x01` = 当前位置；`0x02` = 目标位置 |
| 2–5 | x | int32 | **大端**，米 × 10000 |
| 6–9 | y | int32 | 大端，米 × 10000 |
| 10–13 | yaw | int32 | 大端，弧度 × 10000 |
| 14 | EOF | uint8 | `0x21` |

共 **15 字节**，无校验和，115200 8N1。

例：经过坐标转换后，线上编码值为 `x=1.0 m, y=2.0 m, yaw=0.5 rad`

```
7E 01 00 00 27 10 00 00 4E 20 00 00 13 88 21
   ^ID  ^x=10000   ^y=20000   ^yaw=5000
```

> ⚠ **与 MCU 的对齐状态（2026-10-30 已确认）**：
> 机器人上原来跑的 `SCURM_SentryNavigation/.../send_data.py` 发的是
> **14 字节、没有 ID**（`7E x y yaw 21`，正好是表格里的偏移）。
> 现在统一成**两帧都 15 字节**：当前 `7E 01 x y yaw 21`、目标 `7E 02 x y yaw 21`
> —— **MCU 固件需要同步改成这种格式**，旧 `send_data.py` 必须停掉。
> 若哪天要退回 14 字节，把 `config/mcu_bridge.yaml` 的 `with_id` 改成 `false` 即可，
> 代码不用动（但那样当前/目标两帧就没有区分字节了，MCU 无法分辨）。

## 运行

```bash
# 机器人小电脑上
source env.sh                            # 从仓库根目录执行
ros2 launch mcu_bridge mcu_bridge.launch.py

# 第一次联调：不开串口，只看帧
ros2 launch mcu_bridge mcu_bridge.launch.py dry_run:=true
# 日志里会出现：dry_run：第一帧 7E 01 00 00 27 10 ... 21
```

参数（`config/mcu_bridge.yaml` 全部可改）：启动时先读取 `params_file`，只有显式传入的 `port:=`、`baudrate:=`、坐标系或 `dry_run:=` 才覆盖 YAML。根 `start_robot.sh` 通过 `MCU_PORT` 指定当前机器的串口，默认 `/dev/ttyUSB0`。

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | YAML 中的 by-id 路径 | 串口；可按实际设备修改或用 `port:=` 覆盖 |
| `baudrate` | `115200` | 需与 MCU 一致 |
| `odom_topic` | `/aft_mapped_to_init` | **当前位姿首选来源**（LIO 里程计话题）；置空则只用 TF |
| `odom_timeout` | `0.5` | 里程计超过这么久没更新就退回 TF |
| `output_frame` | `odom` | 发给 MCU 的坐标系（也是目标点换算的目标系） |
| `base_frame` | `base_link` | 退回 TF 时的 `output_frame <- base_frame` |
| `goal_topic` | `/goal_pose` | RViz "2D Goal Pose" 的默认话题 |
| `publish_rate_hz` | `50.0` | 当前位置帧频率 |
| `target_rate_hz` | `10.0` | 目标位置帧频率 |
| `target_hold_sec` | `5.0` | 收到目标后最多再发多少秒（`0` = 永远重复发）；之后停发，MCU 保持最后目标 |
| `with_id` | `true` | 帧里有没有 ID 字节（见上面的 ⚠） |
| `pos_scale` / `ang_scale` | `10000` | 米/弧度 → int32 的缩放 |
| `byte_order` | `big` | 大端 |
| `mount_yaw_deg` | `0.0` | 雷达相对**车头**的安装偏角（deg，逆时针正）；当前位姿和目标点一起旋转 |
| `y_sign` | `-1` | 当前 YAML 将 ROS 的 y 向左转换为线上 y 向右 |
| `yaw_sign` | `1` | MCU 的 yaw 仍是逆时针正，不用翻 |
| `dry_run` | `false` | true = 不开串口，只打第一帧 |
| `enable_velocity` | `false` | 打开则同时发旧的 `0x5A/0xED` 速度帧 |

手动清目标：`ros2 topic pub --once /mcu_bridge/clear std_msgs/msg/Empty {}`

## 历史兼容：SCURM 的 fast_livo

旧机器人外部栈的 fast_livo 帧名是 `camera_init -> aft_mapped`，当时以该帧运行：

```bash
ros2 launch mcu_bridge mcu_bridge.launch.py output_frame:=camera_init
```

当前位姿默认来自里程计话题 `/aft_mapped_to_init`（`odom_topic` 参数），不依赖 TF 帧名；
RViz 的 Fixed Frame 用 `camera_init`，点出来的目标天然就在同一系里。
这是旧栈兼容记录。当前完整入口见 [robot/start_robot.sh](../../robot/start_robot.sh)，使用本仓库 FAST-LIVO2 和 `odom→base_link`，默认 `output_frame=odom`。

## RViz 侧（本机）

1. 本机 `source env.sh`（`ROS_DOMAIN_ID=42`，必须和机器人一致）；
2. 运行根目录 `./rviz_robot.sh`（Fixed Frame = `odom`），匹配默认位置串口链；
3. 工具栏 **2D Goal Pose** 点一个目标 → 发到 `/goal_pose`；
4. 本节点按目标消息的 frame 换算到 `output_frame` 再发。

若同时运行全局重定位，可用 `rviz2 -d robot/rviz/real_nav.rviz` 查看 map 系目标；将目标换算到 odom 需要定位链路提供 `map→odom`。

调试话题：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/mcu_bridge/status` | String | 当前/目标位姿、串口状态、帧计数（1 Hz） |
| `/mcu_bridge/current` | PoseStamped | 发出去的当前位姿（output_frame） |
| `/mcu_bridge/target` | PoseStamped | 发出去的目标位姿（output_frame） |

## 安全行为

- 串口写只在定时器里，订阅回调不碰串口（`/goal_pose` 不会因为串口卡住而丢）。
- **拿不到当前位姿就不发当前帧**（不会拿 (0,0,0) 去骗 MCU 把车往原点拽）。
- **收到第一个目标点之前不发目标帧**。
- 串口打不开/写失败按 `reconnect_interval` 重连，不崩。
- `Ctrl-C` 退出会关串口并打印帧计数。

## 构建 / 部署

本机（开发机）：

```bash
source env.sh
./build.sh nav --packages-select mcu_bridge
```

机器人小电脑（`/home/rc/SCU27RC_Nav`）：

```bash
# 本机执行（只需同步这一个包，不动机器人上其它代码）
./tools/deploy_mcu_bridge.sh
```

脚本将本包同步到目标仓库的 `src/mcu_bridge/`
→ 从远端仓库根目录构建本包
→ 打印 `ros2 pkg prefix mcu_bridge` 和 launch 文件路径。

> 历史机器人上的 `setuptools` 81.x 不支持该工具链的 editable 安装；当前构建入口默认普通安装，沿用该兼容方式。

## 测试

```bash
# ① 帧编解码单测（不需要 ROS、不需要串口）
python3 src/mcu_bridge/test/test_frame.py

# ② PTY 假串口端到端（机器人上跑，不碰真硬件）
python3 src/mcu_bridge/tools/pty_roundtrip.py

# ③ 看串口上到底发了什么（会抢 MCU 的字节，慎用）
ros2 run mcu_bridge mcu_sniff --port /dev/ttyUSB0
```

## 与旧速度帧（`0x5A/0xED`）的关系

`/home/rc/1ARC/mcu_call` 里的 `mcu_serial_bridge` 往同一个串口发 9 字节速度帧
（`0x5A 0x01 vx vy wz 0xED`）。**两个进程写同一个串口会字节级打架**，二选一：

- 只用位置帧：不启动 `mcu_serial_bridge`（本包默认）；
- 两个都要：给本包设 `enable_velocity: true`，它会在同一进程里一起发
  （速度帧频率/缩放/字节序都有参数），然后停掉 `mcu_serial_bridge`。
