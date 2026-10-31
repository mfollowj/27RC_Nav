#!/usr/bin/env python3
"""帧编解码单元测试：python3 test/test_frame.py（不依赖 ROS / 串口）。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mcu_bridge.frame import (  # noqa: E402
    EOF_POS, SOF_POS, build_position_frame, build_velocity_frame,
    decode_position_frame, decode_velocity_frame, to_int32)


def test_current_frame_vector():
    """1.0, 2.0, 0.5 rad @ scale 10000，大端，带 ID。"""
    frame = build_position_frame(0x01, 1.0, 2.0, 0.5)
    assert len(frame) == 15, len(frame)
    assert frame == bytes.fromhex('7E 01 00002710 00004E20 00001388 21'), frame.hex()


def test_target_frame_vector():
    frame = build_position_frame(0x02, -1.0, 0.0, -0.5)
    assert frame[0] == 0x7E and frame[1] == 0x02 and frame[-1] == 0x21
    x, y, yaw = decode_position_frame(frame)[1:]
    assert (x, y, yaw) == (-10000, 0, -5000), (x, y, yaw)


def test_without_id_matches_table_offsets():
    frame = build_position_frame(0x01, 1.0, 2.0, 0.5, with_id=False)
    assert len(frame) == 14
    assert frame == bytes.fromhex('7E 00002710 00004E20 00001388 21'), frame.hex()
    assert decode_position_frame(frame, with_id=False)[0] is None


def test_little_endian():
    frame = build_position_frame(0x01, 1.0, 2.0, 0.5, byte_order='little')
    assert frame[2:6] == bytes.fromhex('10270000'), frame.hex()
    assert decode_position_frame(frame, byte_order='little')[1] == 10000


def test_saturation_and_nan():
    assert to_int32(1e9, 10000.0) == 2147483647
    assert to_int32(-1e9, 10000.0) == -2147483648
    assert to_int32(float('nan')) == 0


def test_rounding_half_away_from_zero():
    # 0.00005 m * 10000 = 0.5 -> 1；-0.5 -> -1
    assert to_int32(0.00005, 10000.0) == 1
    assert to_int32(-0.00005, 10000.0) == -1


def test_velocity_frame_vector():
    """与 mcu_call README 的例子一致：1.0, 2.0, 0.5 -> 5A 01 E8 03 D0 07 F4 01 ED"""
    frame = build_velocity_frame(1.0, 2.0, 0.5)
    assert frame == bytes.fromhex('5A 01 E8 03 D0 07 F4 01 ED'), frame.hex()
    assert decode_velocity_frame(frame) == (1000, 2000, 500)


def test_bad_frame_raises():
    for bad in (b'', b'\x7E' * 14, build_position_frame(1, 0, 0, 0)[:-1] + b'\x00'):
        try:
            decode_position_frame(bad)
        except ValueError:
            continue
        raise AssertionError('should have raised: %r' % bad)


if __name__ == '__main__':
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('PASS %s' % name)
            except Exception as exc:      # noqa: BLE001
                failed += 1
                print('FAIL %s: %s' % (name, exc))
    print('---')
    print('%s' % ('ALL PASS' if failed == 0 else '%d FAILED' % failed))
    sys.exit(1 if failed else 0)
