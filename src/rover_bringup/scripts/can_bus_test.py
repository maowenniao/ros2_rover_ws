#!/usr/bin/env python3
"""
can_bus_test.py - 诊断 ODrive CAN 链路是否正常

用法：
    python3 can_bus_test.py can0

期望输出：
    - 列出当前 ODrive 心跳帧
    - 显示每个轴的 axis_state 和 error 标志
"""
import sys
import struct
import time

import can


CMD_HEARTBEAT = 0x01
NODE_ID_MASK = 0x3F
CMD_SHIFT = 6


def parse_can_id(arb_id):
    node_id = arb_id & NODE_ID_MASK
    cmd = (arb_id >> CMD_SHIFT) & 0x3F
    return cmd, node_id


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else 'can0'

    bus = can.interface.Bus(
        interface='socketcan', channel=iface, bitrate=250000
    )

    print(f'[{iface}] 监听 CAN 帧（按 Ctrl-C 退出）...')
    print(f'等待 ODrive 心跳（cmd=0x01）...')
    print('-' * 60)

    seen = set()
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            cmd, nid = parse_can_id(msg.arbitration_id)

            if cmd == CMD_HEARTBEAT and len(msg.data) >= 8:
                axis_state = struct.unpack('<I', msg.data[0:4])[0]
                error_flags = struct.unpack('<I', msg.data[4:8])[0]
                tag = f'nid={nid}'
                if tag not in seen:
                    print(f'发现 ODrive: {tag}')
                    seen.add(tag)
                # 简洁打印
                ts = time.strftime('%H:%M:%S')
                state_str = {
                    0: 'UNDEFINED',
                    1: 'IDLE',
                    2: 'STARTUP',
                    3: 'EARLY_BOOT',
                    4: 'LATE_BOOT',
                    7: 'CALIBRATION',
                    8: 'CLOSED_LOOP',
                }.get(axis_state, f'STATE={axis_state}')
                if error_flags == 0:
                    err_str = '无错'
                else:
                    err_str = f'错误=0x{error_flags:08X}'
                print(f'[{ts}] {tag} state={state_str:<14} {err_str}')
    except KeyboardInterrupt:
        pass
    finally:
        print('\n退出')


if __name__ == '__main__':
    main()