"""MCU 串口帧编解码（纯逻辑，不 import ROS/串口，方便单测）。

两套帧并存，靠**帧头**区分，互不影响：

1) 位置帧（新协议，本包主力）—— 0x7E ... 0x21
   ------------------------------------------------------------
   offset  field  type     meaning
   0       SOF    uint8    0x7E
   1       ID     uint8    0x01 = 当前位置 / 0x02 = 目标位置
   2..5    x      int32    大端，米 × pos_scale（默认 10000）
   6..9    y      int32    大端，米 × pos_scale
   10..13  yaw    int32    大端，弧度 × ang_scale（默认 10000）
   14      EOF    uint8    0x21

   共 15 字节。`with_id=False` 时**没有 ID 字节**：x 从字节 1 开始、EOF 在字节 13
   （共 14 字节），与协议表格里 1~4 / 5~8 / 9~12 / 13 的偏移完全一致。
   两种都实现是因为文档的偏移表没给 ID 字节留位置，而文字说明又明确写了
   "帧头后面 0x01/0x02 区分当前/目标" —— 用 with_id 开关兼容，默认 True。

2) 速度帧（旧协议，/home/rc/1ARC/mcu_call 那套）—— 0x5A ... 0xED
   ------------------------------------------------------------
   0x5A | 0x01 | vx int16 | vy int16 | wz int16 | 0xED（9 字节，小端，×1000）
   默认不开启（enable_velocity=false），需要"一个进程同时发位置和速度"时再开。

多字节字段字节序默认**大端**（位置帧）/ 小端（速度帧），都可用参数切。
浮点转整数用"四舍五入、远离零"（与他原来的 frame.py 一致），并做饱和。
"""

import math
import struct

# ---- 位置帧 ---------------------------------------------------------------
SOF_POS = 0x7E
EOF_POS = 0x21
MSG_ID_CURRENT = 0x01
MSG_ID_TARGET = 0x02

POS_FRAME_SIZE_WITH_ID = 15
POS_FRAME_SIZE_NO_ID = 14

# ---- 速度帧（旧协议，保留兼容） -------------------------------------------
SOF_VEL = 0x5A
EOF_VEL = 0xED
MSG_ID_VELOCITY = 0x01
VEL_FRAME_SIZE = 9

INT32_MIN = -2147483648
INT32_MAX = 2147483647
INT16_MIN = -32768
INT16_MAX = 32767

VALID_BYTE_ORDERS = ("little", "big")


def round_half_away_from_zero(value):
    """四舍五入（0.5 -> 1，-0.5 -> -1），与 MCU 侧 C 的 (int)(x + 0.5) 语义接近。"""
    if value >= 0.0:
        return math.floor(value + 0.5)
    return math.ceil(value - 0.5)


def _quantize(value, scale, lo, hi, what="value"):
    """value * scale -> 整数，饱和到 [lo, hi]；NaN 当 0，inf 饱和。"""
    scaled = float(value) * float(scale)
    if math.isnan(scaled):
        return 0
    if math.isinf(scaled):
        return hi if scaled > 0.0 else lo
    rounded = round_half_away_from_zero(scaled)
    if rounded > hi:
        return hi
    if rounded < lo:
        return lo
    return int(rounded)


def to_int32(value, scale=1.0):
    return _quantize(value, scale, INT32_MIN, INT32_MAX)


def to_int16(value, scale=1.0):
    return _quantize(value, scale, INT16_MIN, INT16_MAX)


def _byte_order_prefix(byte_order):
    if byte_order not in VALID_BYTE_ORDERS:
        raise ValueError(
            "byte_order must be one of %r, got %r"
            % (VALID_BYTE_ORDERS, byte_order))
    return ">" if byte_order == "big" else "<"


# ---------------------------------------------------------------- 位置帧
def build_position_frame(msg_id, x, y, yaw,
                         pos_scale=10000.0, ang_scale=10000.0,
                         byte_order="big", with_id=True,
                         sof=SOF_POS, eof=EOF_POS):
    """打包一帧位置帧（x/y 米，yaw 弧度），返回 bytes。"""
    prefix = _byte_order_prefix(byte_order)
    payload = struct.pack(
        prefix + "iii",
        to_int32(x, pos_scale),
        to_int32(y, pos_scale),
        to_int32(yaw, ang_scale),
    )
    head = bytes((sof & 0xFF,))
    if with_id:
        head += bytes((int(msg_id) & 0xFF,))
    return head + payload + bytes((eof & 0xFF,))


def decode_position_frame(frame, byte_order="big", with_id=True,
                          sof=SOF_POS, eof=EOF_POS):
    """校验并解析位置帧 -> (msg_id, x_raw, y_raw, yaw_raw)。

    x_raw/y_raw/yaw_raw 是**已缩放后的整数**，除以 pos_scale/ang_scale 才是米/弧度。
    with_id=False 时 msg_id 返回 None。
    """
    prefix = _byte_order_prefix(byte_order)
    expect = POS_FRAME_SIZE_WITH_ID if with_id else POS_FRAME_SIZE_NO_ID
    if frame is None or len(frame) != expect:
        raise ValueError("frame must be exactly %d bytes, got %r"
                         % (expect, len(frame) if frame is not None else None))
    if frame[0] != (sof & 0xFF):
        raise ValueError("bad SOF 0x%02X" % frame[0])
    if frame[-1] != (eof & 0xFF):
        raise ValueError("bad EOF 0x%02X" % frame[-1])
    if with_id:
        msg_id = frame[1]
        payload = frame[2:-1]
    else:
        msg_id = None
        payload = frame[1:-1]
    x, y, yaw = struct.unpack(prefix + "iii", payload)
    return msg_id, x, y, yaw


# ---------------------------------------------------------------- 速度帧
def build_velocity_frame(vx, vy, wz,
                         linear_scale=1000.0, angular_scale=1000.0,
                         byte_order="little"):
    """旧协议速度帧：5A 01 vx vy wz ED（9 字节）。"""
    prefix = _byte_order_prefix(byte_order)
    payload = struct.pack(
        prefix + "hhh",
        to_int16(vx, linear_scale),
        to_int16(vy, linear_scale),
        to_int16(wz, angular_scale),
    )
    return bytes((SOF_VEL, MSG_ID_VELOCITY)) + payload + bytes((EOF_VEL,))


def decode_velocity_frame(frame, byte_order="little"):
    if frame is None or len(frame) != VEL_FRAME_SIZE:
        raise ValueError("frame must be exactly %d bytes" % VEL_FRAME_SIZE)
    if frame[0] != SOF_VEL:
        raise ValueError("bad SOF 0x%02X" % frame[0])
    if frame[1] != MSG_ID_VELOCITY:
        raise ValueError("unexpected message id 0x%02X" % frame[1])
    if frame[8] != EOF_VEL:
        raise ValueError("bad EOF 0x%02X" % frame[8])
    prefix = _byte_order_prefix(byte_order)
    return struct.unpack(prefix + "hhh", frame[2:8])
