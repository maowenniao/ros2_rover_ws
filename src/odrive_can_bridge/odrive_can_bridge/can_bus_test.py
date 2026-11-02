#!/usr/bin/env python3
"""
can_bus_test.py
===============

ODrive CAN 总线测试工具。

功能：
    - 扫描 CAN 总线上的 ODrive 设备
    - 发送心跳检测
    - 发送速度命令测试
    - 显示 ODrive 状态和诊断信息
    - 交互式命令行界面

使用方式：
    # 交互式测试
    ros2 run odrive_can_bridge can_bus_test.py
    
    # 或直接运行
    python3 odrive_can_bridge/can_bus_test.py

依赖：
    python-can
    ros2 (可选，用于发布诊断信息)
"""
import struct
import time
import sys
import os
from typing import List, Dict, Optional

# 尝试导入 python-can
try:
    import can
except ImportError:
    print("[ERROR] 缺少 python-can 库")
    print("安装: pip install python-can")
    sys.exit(1)

from odrive_can_bridge.can_protocol import (
    CMD_CLEAR_ERRORS,
    CMD_ESTOP,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CMD_HEARTBEAT,
    CMD_REBOOT,
    CMD_SET_AXIS_STATE,
    CMD_SET_CONTROLLER_MODE,
    CMD_SET_INPUT_POS,
    CMD_SET_INPUT_TORQUE,
    CMD_SET_INPUT_VEL,
    CMD_SET_LIMITS,
    decode_encoder_estimates,
    decode_heartbeat,
    make_can_id,
    parse_can_id,
)

# Axis State
AXIS_STATE_IDLE = 0x01
AXIS_STATE_CLOSED_LOOP_CONTROL = 0x08

# 轴状态名称
AXIS_STATE_NAMES = {
    0x00: "UNDEFINED",
    0x01: "IDLE",
    0x02: "STARTUP_SEQUENCE",
    0x03: "FULL_CALIBRATION",
    0x04: "MOTOR_CALIBRATION",
    0x06: "ENCODER_INDEX_SEARCH",
    0x07: "ENCODER_CALIBRATION",
    0x08: "CLOSED_LOOP_CONTROL",
    0x0B: "LOCK_IN_SPINOUT",
}

# 控制模式
CONTROL_MODE_VOLTAGE = 0x00
CONTROL_MODE_TORQUE = 0x01
CONTROL_MODE_VELOCITY = 0x02
CONTROL_MODE_POSITION = 0x03

CONTROL_MODE_NAMES = {
    CONTROL_MODE_VOLTAGE: "VOLTAGE",
    CONTROL_MODE_TORQUE: "TORQUE",
    CONTROL_MODE_VELOCITY: "VELOCITY",
    CONTROL_MODE_POSITION: "POSITION",
}

# ======================================================================
# ODrive 错误码
# ======================================================================
AXIS_ERROR_NAMES = {
    0x00000001: "Phase Resistance ADC Fail",
    0x00000002: "Boottime ADC Non-Zero",
    0x00000004: "System Level",
    0x00000008: "Motor Wrong HW",
    0x00000010: "Wrong Axis ID",
    0x00000020: "Invalid BEMF Conv Method",
    0x00000040: "Ctrl Tick Not Running",
    0x00000080: "Stopped Without Estimate",
    0x00000100: "Periodic Callback Not Running",
    0x00000200: "Missing Estimate",
    0x00000400: "Override Current Setpoint",
    0x00000800: "Modulation D Zero",
    0x00001000: "Motor Calibration Failed",
}

MOTOR_ERROR_NAMES = {
    0x00000001: "Boottime Sensors",
    0x00000002: "Phase Resistance OOR",
    0x00000004: "Phase Inductance OOR",
    0x00000008: "Drivers Over Temp",
    0x00000010: "Failed Setup HW",
    0x00001000: "Power Stage Mismatch",
}

ENCODER_ERROR_NAMES = {
    0x00000001: "Boottime Not Settled",
    0x00000002: "Index Not Found",
    0x00000008: "Calibration Ratio OOR",
    0x00000040: "Comm Overflow",
    0x00000080: "Missing Estimates",
}

CONTROLLER_ERROR_NAMES = {
    0x00000001: "Overspeed",
    0x00000002: "Traj Bad Input Pos",
    0x00000004: "Traj Bad Input Vel",
    0x00001000: "Watchdog Rejected Cmd",
}


def decode_errors(error_flag: int, error_names: dict) -> List[str]:
    """解析错误标志."""
    errors = []
    for flag, name in error_names.items():
        if error_flag & flag:
            errors.append(name)
    return errors if errors else ["None"]


# ======================================================================
# ODrive 状态追踪
# ======================================================================
class ODriveDevice:
    """单个 ODrive 设备状态."""

    def __init__(self, node_id: int):
        self.node_id = node_id
        self.last_heartbeat = 0.0
        self.axis_state = 0
        self.axis_error = 0
        self.motor_error = 0
        self.encoder_error = 0
        self.controller_error = 0
        self.pos_estimate = 0.0
        self.vel_estimate = 0.0
        self.bus_voltage = 0.0
        self.bus_current = 0.0
        self.is_online = False
        self.heartbeat_count = 0

    @property
    def is_in_error(self) -> bool:
        return self.axis_error != 0 or self.motor_error != 0 or \
               self.encoder_error != 0 or self.controller_error != 0

    @property
    def is_closed_loop(self) -> bool:
        return self.axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL

    @property
    def uptime(self) -> float:
        return time.time() - self.last_heartbeat if self.last_heartbeat > 0 else 0.0


# ======================================================================
# CAN 总线测试器
# ======================================================================
class ODriveCanTester:
    """ODrive CAN 总线测试器."""

    def __init__(self, can_channel: str = 'can0', bitrate: int = 250000,
                 node_ids: List[int] = None):
        self.can_channel = can_channel
        self.bitrate = bitrate
        self.node_ids = node_ids or [0, 1, 2, 3]
        self.devices: Dict[int, ODriveDevice] = {
            nid: ODriveDevice(nid) for nid in self.node_ids
        }
        self.bus = None
        self.running = False
        self.last_recv_time = 0.0

    def connect(self) -> bool:
        """连接 CAN 总线."""
        try:
            self.bus = can.interface.Bus(
                interface='socketcan',
                channel=self.can_channel,
                bitrate=self.bitrate,
            )
            print(f"[OK] CAN 总线连接: {self.can_channel} @ {self.bitrate} bps")
            return True
        except Exception as e:
            print(f"[ERROR] CAN 总线连接失败: {e}")
            return False

    def disconnect(self):
        """断开 CAN 总线."""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            print("[OK] CAN 总线已断开")

    def send_cmd(self, node_id: int, cmd_id: int, data: bytes = b'') -> bool:
        """发送 CAN 命令."""
        if not self.bus:
            print("[ERROR] CAN 总线未连接")
            return False
        try:
            msg = can.Message(
                arbitration_id=make_can_id(node_id, cmd_id),
                data=data,
                is_extended_id=False,
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"[ERROR] 发送失败: {e}")
            return False

    def scan_devices(self) -> List[int]:
        """扫描 CAN 总线上的 ODrive 设备."""
        print("\n" + "=" * 60)
        print("扫描 CAN 总线上的 ODrive 设备...")
        print("=" * 60)

        # 发送心跳请求（实际上是读取编码器估计）
        found = []
        for nid in range(64):  # 扫描所有 64 个可能的节点 ID
            # 请求编码器估计
            self.send_cmd(nid, CMD_GET_ENCODER_ESTIMATES)
            time.sleep(0.01)

        # 接收心跳消息来检测在线设备
        print("\n监听心跳消息 (5 秒)...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            msg = self.bus.recv(0.1)
            if msg and msg.arbitration_id > 0:
                cmd_id, node_id = parse_can_id(msg.arbitration_id)
                if node_id not in found:
                    found.append(node_id)
                    self._parse_heartbeat(node_id, msg)
                    print(f"  发现 ODrive nid={node_id}")

        print(f"\n发现 {len(found)} 个 ODrive 设备: {found}")
        return found

    def _parse_heartbeat(self, node_id: int, msg) -> None:
        """解析心跳消息."""
        if node_id not in self.devices:
            self.devices[node_id] = ODriveDevice(node_id)
        dev = self.devices[node_id]

        dev.last_heartbeat = time.time()
        dev.is_online = True
        dev.heartbeat_count += 1

        if len(msg.data) >= 8:
            hb = decode_heartbeat(bytes(msg.data))
            dev.axis_state = hb.axis_state
            dev.axis_error = hb.axis_error
            dev.motor_error = int(hb.motor_error)

    def _parse_encoder(self, node_id: int, msg) -> None:
        """解析编码器估计消息."""
        if node_id not in self.devices:
            return
        dev = self.devices[node_id]
        if len(msg.data) >= 8:
            dev.pos_estimate, dev.vel_estimate = decode_encoder_estimates(bytes(msg.data))

    def _parse_bus_voltage(self, node_id: int, msg) -> None:
        """解析总线电压/电流消息."""
        if node_id not in self.devices:
            return
        dev = self.devices[node_id]
        if len(msg.data) >= 8:
            dev.bus_voltage, dev.bus_current = struct.unpack('<ff', msg.data[0:8])

    def show_device_status(self, node_id: int) -> None:
        """显示单个设备状态."""
        if node_id not in self.devices:
            print(f"  设备 nid={node_id} 不存在")
            return
        dev = self.devices[node_id]

        print(f"\n{'='*50}")
        print(f"ODrive Node ID: {node_id}")
        print(f"{'='*50}")
        print(f"  在线状态: {'[在线]' if dev.is_online else '[离线]'}")
        print(f"  轴状态: {AXIS_STATE_NAMES.get(dev.axis_state, f'0x{dev.axis_state:X}')}")
        print(f"  运行时间: {dev.uptime:.1f}s")
        print(f"  心跳计数: {dev.heartbeat_count}")

        if dev.is_online:
            print(f"\n  编码器:")
            print(f"    位置: {dev.pos_estimate:.4f} turns")
            print(f"    速度: {dev.vel_estimate:.4f} turn/s")

            if dev.bus_voltage > 0:
                print(f"\n  电源:")
                print(f"    总线电压: {dev.bus_voltage:.2f} V")
                print(f"    总线电流: {dev.bus_current:.2f} A")

        if dev.is_in_error:
            print(f"\n  [错误]:")
            axis_errs = decode_errors(dev.axis_error, AXIS_ERROR_NAMES)
            motor_errs = decode_errors(dev.motor_error, MOTOR_ERROR_NAMES)
            print(f"    轴错误: {', '.join(axis_errs)}")
            print(f"    电机错误: {', '.join(motor_errs)}")

    def show_all_status(self) -> None:
        """显示所有设备状态."""
        print("\n" + "=" * 60)
        print("ODrive 设备状态总览")
        print("=" * 60)
        for nid in self.devices:
            self.show_device_status(nid)

    def test_velocity_cmd(self, node_id: int, vel_rps: float = 1.0,
                          duration: float = 3.0) -> None:
        """测试速度命令."""
        if node_id not in self.devices:
            print(f"[ERROR] 设备 nid={node_id} 不存在")
            return

        print(f"\n测试速度命令: nid={node_id}, vel={vel_rps} turn/s, 持续 {duration}s")

        # 设置控制模式为速度模式
        self.send_cmd(node_id, CMD_SET_CONTROLLER_MODE,
                      struct.pack('<II', CONTROL_MODE_VELOCITY, 0x01))
        time.sleep(0.1)

        # 进入闭环控制
        self.send_cmd(node_id, CMD_SET_AXIS_STATE,
                      struct.pack('<I', AXIS_STATE_CLOSED_LOOP_CONTROL))
        print(f"  已发送进入闭环控制命令")
        time.sleep(0.5)

        # 发送速度命令
        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                self.send_cmd(node_id, CMD_SET_INPUT_VEL,
                              struct.pack('<ff', vel_rps, 0.0))
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass

        # 停止
        self.send_cmd(node_id, CMD_SET_INPUT_VEL,
                      struct.pack('<ff', 0.0, 0.0))
        print("  已发送停止命令")

    def test_full_sequence(self, node_ids: List[int] = None) -> None:
        """完整测试序列."""
        nids = node_ids or self.node_ids
        print("\n" + "=" * 60)
        print("完整 ODrive 测试序列")
        print("=" * 60)

        for nid in nids:
            print(f"\n--- 测试 ODrive nid={nid} ---")

            # 1. 检查是否在线（等待心跳）
            time.sleep(1.0)
            if nid in self.devices and self.devices[nid].is_online:
                print(f"  [OK] 设备在线")
                self.show_device_status(nid)
            else:
                print(f"  [WARN] 设备未响应心跳，继续尝试...")

            # 2. 清错
            self.send_cmd(nid, CMD_CLEAR_ERRORS)
            print(f"  已发送清错命令")
            time.sleep(0.1)

            # 3. 设置控制模式
            self.send_cmd(nid, CMD_SET_CONTROLLER_MODE,
                          struct.pack('<II', CONTROL_MODE_VELOCITY, 0x01))
            print(f"  已设置速度控制模式")
            time.sleep(0.1)

            # 4. 进入闭环
            self.send_cmd(nid, CMD_SET_AXIS_STATE,
                          struct.pack('<I', AXIS_STATE_CLOSED_LOOP_CONTROL))
            print(f"  已发送进入闭环控制命令")
            time.sleep(1.0)

            # 5. 测试速度命令
            if nid in self.devices and self.devices[nid].is_closed_loop:
                print(f"  [OK] 已进入闭环控制")
                self.test_velocity_cmd(nid, vel_rps=0.5, duration=2.0)
            else:
                print(f"  [WARN] 未进入闭环，跳过速度测试")

            # 6. 返回 idle
            self.send_cmd(nid, CMD_SET_AXIS_STATE,
                          struct.pack('<I', AXIS_STATE_IDLE))
            print(f"  已发送回到 IDLE 命令")
            time.sleep(0.2)

        print("\n完整测试序列结束")

    def receive_loop(self, callback=None) -> None:
        """接收 CAN 消息循环."""
        print("\n开始接收 CAN 消息 (Ctrl+C 退出)...")
        try:
            while self.running:
                msg = self.bus.recv(0.1)
                if msg:
                    self.last_recv_time = time.time()
                    cmd_id, node_id = parse_can_id(msg.arbitration_id)

                    if callback:
                        callback(node_id, msg)
                    else:
                        self._default_recv_handler(node_id, msg)
        except KeyboardInterrupt:
            print("\n接收停止")

    def _default_recv_handler(self, node_id: int, msg) -> None:
        """默认接收处理."""
        cmd_id, _ = parse_can_id(msg.arbitration_id)

        if cmd_id == CMD_HEARTBEAT:
            self._parse_heartbeat(node_id, msg)
        elif cmd_id == CMD_GET_ENCODER_ESTIMATES:
            self._parse_encoder(node_id, msg)
        elif cmd_id == CMD_GET_BUS_VOLTAGE_CURRENT:
            self._parse_bus_voltage(node_id, msg)

        # 打印接收到的消息
        arb_id = msg.arbitration_id
        data_str = ' '.join(f'{b:02X}' for b in msg.data[:8])
        print(f"  RX: arb_id=0x{arb_id:03X} ({cmd_id:02X}, nid={node_id}) "
              f"data=[{data_str}]")


# ======================================================================
# 交互式命令行
# ======================================================================
class InteractiveCLI:
    """交互式命令行界面."""

    def __init__(self, tester: ODriveCanTester):
        self.tester = tester
        self.running = True

    def run(self):
        """运行交互式界面."""
        print("\n" + "=" * 60)
        print("  ODrive CAN 总线测试工具 - 交互式模式")
        print("=" * 60)
        print()
        print("命令列表:")
        print("  scan              - 扫描 CAN 总线上的 ODrive 设备")
        print("  status [nid]      - 显示设备状态（可选指定 nid）")
        print("  status_all        - 显示所有设备状态")
        print("  heartbeat [nid]   - 发送心跳请求")
        print("  clear_error nid   - 清除错误")
        print("  setup nid         - 完整设置设备（模式+闭环）")
        print("  vel_test nid v s  - 速度测试 (v=速度 turn/s, s=秒)")
        print("  full_test [nids]  - 完整测试序列")
        print("  send_cmd nid cmd [data] - 发送自定义命令")
        print("  rx                - 开始接收 CAN 消息")
        print("  quit/exit/q       - 退出")
        print()

        while self.running:
            try:
                cmd = input("odrive-test> ").strip().split()
                if not cmd:
                    continue
                self._handle_command(cmd)
            except EOFError:
                break
            except KeyboardInterrupt:
                print("\n(Ctrl+C 退出)")

    def _handle_command(self, parts: List[str]):
        """处理命令."""
        cmd = parts[0].lower()
        args = parts[1:]

        if cmd in ('quit', 'exit', 'q'):
            self.running = False
        elif cmd == 'scan':
            self.tester.scan_devices()
        elif cmd == 'status':
            nid = int(args[0]) if args else None
            if nid:
                self.tester.show_device_status(nid)
            else:
                self.tester.show_all_status()
        elif cmd == 'status_all':
            self.tester.show_all_status()
        elif cmd == 'heartbeat':
            nid = int(args[0]) if args else 0
            # 心跳是 ODrive 主动发送的，这里发送编码器请求
            self.tester.send_cmd(nid, CMD_GET_ENCODER_ESTIMATES)
            print(f"  已发送 nid={nid} 编码器请求")
        elif cmd == 'clear_error':
            nid = int(args[0]) if args else 0
            self.tester.send_cmd(nid, CMD_CLEAR_ERRORS)
            print(f"  已发送 nid={nid} 清错命令")
        elif cmd == 'setup':
            nid = int(args[0]) if args else 0
            self.tester.send_cmd(nid, CMD_CLEAR_ERRORS)
            time.sleep(0.05)
            self.tester.send_cmd(nid, CMD_SET_CONTROLLER_MODE,
                                 struct.pack('<II', CONTROL_MODE_VELOCITY, 0x01))
            time.sleep(0.05)
            self.tester.send_cmd(nid, CMD_SET_AXIS_STATE,
                                 struct.pack('<I', AXIS_STATE_CLOSED_LOOP_CONTROL))
            print(f"  已设置 nid={nid} 为速度闭环控制")
        elif cmd == 'vel_test':
            if len(args) < 3:
                print("  用法: vel_test nid velocity(seconds)")
                return
            nid = int(args[0])
            vel = float(args[1])
            dur = float(args[2])
            self.tester.test_velocity_cmd(nid, vel, dur)
        elif cmd == 'full_test':
            nids = [int(x) for x in args] if args else None
            self.tester.test_full_sequence(nids)
        elif cmd == 'send_cmd':
            if len(args) < 2:
                print("  用法: send_cmd nid cmd_id [data_hex]")
                return
            nid = int(args[0])
            cmd_id = int(args[1], 16) if args[1].startswith('0x') else int(args[1])
            data = bytes()
            if len(args) > 2:
                hex_str = args[2].replace(' ', '')
                data = bytes.fromhex(hex_str)
            self.tester.send_cmd(nid, cmd_id, data)
            print(f"  已发送: nid={nid}, cmd=0x{cmd_id:02X}, data={data.hex()}")
        elif cmd == 'rx':
            self.tester.running = True
            self.tester.receive_loop()
        else:
            print(f"  未知命令: {cmd}")
            print("  输入 help 查看命令列表")


# ======================================================================
# 主函数
# ======================================================================
def main():
    """主函数."""
    import argparse

    parser = argparse.ArgumentParser(description='ODrive CAN 总线测试工具')
    parser.add_argument('--channel', '-c', default='can0',
                        help='CAN 通道 (默认: can0)')
    parser.add_argument('--bitrate', '-b', type=int, default=250000,
                        help='CAN 波特率 (默认: 250000)')
    parser.add_argument('--node-ids', '-n', type=int, nargs='+',
                        default=[0, 1, 2, 3],
                        help='ODrive 节点 ID 列表')
    parser.add_argument('--mode', '-m', choices=['interactive', 'scan', 'test'],
                        default='interactive',
                        help='运行模式 (默认: interactive)')
    parser.add_argument('--duration', '-d', type=float, default=2.0,
                        help='测试持续时间 (秒)')

    args = parser.parse_args()

    # 创建测试器
    tester = ODriveCanTester(
        can_channel=args.channel,
        bitrate=args.bitrate,
        node_ids=args.node_ids,
    )

    if not tester.connect():
        sys.exit(1)

    try:
        if args.mode == 'interactive':
            cli = InteractiveCLI(tester)
            cli.run()
        elif args.mode == 'scan':
            tester.scan_devices()
        elif args.mode == 'test':
            tester.test_full_sequence()
    finally:
        tester.disconnect()


if __name__ == '__main__':
    main()