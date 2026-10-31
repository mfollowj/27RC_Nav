#!/usr/bin/env python3
"""串口嗅探：把 MCU 串口上收到的字节流按 0x7E/0x21 位置帧解析打印。

⚠ 串口是"共享"的：这个工具读走的字节，MCU 就收不到了。只在
  ① 拔掉 MCU；② 用 PTY 假串口；③ MCU 已停止解析 的时候用。

    # 用 PTY 假串口联调（不碰真硬件）
    python3 -m mcu_bridge.sniff --port /dev/pts/5

    # 看真串口（会抢 MCU 的数据，慎用）
    ros2 run mcu_bridge mcu_sniff --port /dev/ttyUSB0 --baudrate 115200
"""
import argparse
import sys

from mcu_bridge.frame import (MSG_ID_CURRENT, MSG_ID_TARGET, POS_FRAME_SIZE_NO_ID,
                              POS_FRAME_SIZE_WITH_ID, SOF_POS,
                              decode_position_frame)

_ID_NAME = {MSG_ID_CURRENT: 'current', MSG_ID_TARGET: 'target'}


def frames_from_buffer(buf, with_id=True):
    """从字节流里切出完整帧，返回 (frames, rest)。"""
    size = POS_FRAME_SIZE_WITH_ID if with_id else POS_FRAME_SIZE_NO_ID
    out = []
    while True:
        idx = buf.find(bytes((SOF_POS,)))
        if idx < 0:
            return out, b''
        buf = buf[idx:]
        if len(buf) < size:
            return out, buf
        cand = buf[:size]
        try:
            out.append(decode_position_frame(cand, with_id=with_id))
            buf = buf[size:]
        except ValueError:
            buf = buf[1:]          # 帧头误判，跳过这个字节继续找
    return out, buf


def main(argv=None):
    ap = argparse.ArgumentParser(description='嗅探 0x7E/0x21 位置帧')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baudrate', type=int, default=115200)
    ap.add_argument('--no-id', action='store_true', help='帧里没有 ID 字节（14 字节）')
    ap.add_argument('--pos-scale', type=float, default=10000.0)
    ap.add_argument('--ang-scale', type=float, default=10000.0)
    args, _ = ap.parse_known_args(argv)

    import serial
    port = serial.Serial(args.port, args.baudrate, timeout=0.2)
    print('sniffing %s @ %d ...' % (args.port, args.baudrate))
    buf = b''
    try:
        while True:
            buf += port.read(256)
            frames, buf = frames_from_buffer(buf, with_id=not args.no_id)
            for msg_id, x, y, yaw in frames:
                name = _ID_NAME.get(msg_id, str(msg_id))
                print('%-7s x=%8.4f y=%8.4f yaw=%8.4f rad (%7.2f deg)'
                      % (name, x / args.pos_scale, y / args.pos_scale,
                         yaw / args.ang_scale, yaw / args.ang_scale * 57.29578))
    except KeyboardInterrupt:
        pass
    finally:
        port.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
