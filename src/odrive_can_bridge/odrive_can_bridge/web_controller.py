#!/usr/bin/env python3
"""
web_controller.py
=================

ODrive 小车 Web 控制服务器。

功能：
    - 提供 Web 界面控制小车（前进、后退、左转、右转、停止）
    - 速度调节（滑块）
    - 模式切换（速度模式/位置模式）
    - 急停按钮
    - ODrive 状态实时监控
    - 设备扫描和设置

使用方式：
    ros2 run odrive_can_bridge web_controller.py
    或
    python3 odrive_can_bridge/web_controller.py

    然后浏览器访问: http://<jetson-ip>:5000
"""
import os
import sys
import struct
import time
import threading
import platform
from typing import Dict, Optional

# 尝试导入所需库
try:
    import can
except ImportError:
    print("[ERROR] 缺少 python-can 库")
    print("安装: pip install python-can")
    sys.exit(1)

try:
    from flask import Flask, render_template, request, jsonify, Response
except ImportError:
    print("[ERROR] 缺少 Flask 库")
    print("安装: pip install flask")
    sys.exit(1)

# ============================================================
# ODrive CAN 协议常量
# ============================================================
CMD_HEARTBEAT = 0x01
CMD_ESTOP = 0x02
CMD_SET_AXIS_STATE = 0x06
CMD_GET_ENCODER_ESTIMATES = 0x07
CMD_SET_CONTROLLER_MODE = 0x09
CMD_SET_INPUT_VEL = 0x0C
CMD_SET_INPUT_POS = 0x0D
CMD_SET_INPUT_TORQUE = 0x0E
CMD_SET_LIMITS = 0x10
CMD_GET_IQ = 0x18
CMD_REBOOT = 0x1C
CMD_GET_BUS_VOLTAGE_CURRENT = 0x1D
CMD_CLEAR_ERRORS = 0x1E

AXIS_STATE_IDLE = 0x01
AXIS_STATE_CLOSED_LOOP_CONTROL = 0x08

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

CONTROL_MODE_VELOCITY = 0x02
CONTROL_MODE_POSITION = 0x03

NODE_ID_MASK = 0x3F
CMD_ID_SHIFT = 6


def make_can_id(node_id: int, cmd_id: int) -> int:
    """根据 ODrive 协议组装 CAN 帧 ID."""
    return ((cmd_id & 0x3F) << CMD_ID_SHIFT) | (node_id & NODE_ID_MASK)


def parse_can_id(arb_id: int):
    """解析 arb_id 返回 (cmd_id, node_id)."""
    node_id = arb_id & NODE_ID_MASK
    cmd_id = (arb_id >> CMD_ID_SHIFT) & 0x3F
    return cmd_id, node_id


# ============================================================
# ODrive 设备状态
# ============================================================
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
        return self.axis_error != 0 or self.motor_error != 0

    @property
    def is_closed_loop(self) -> bool:
        return self.axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL

    def to_dict(self) -> dict:
        return {
            "node_id": self.node_id,
            "is_online": self.is_online,
            "axis_state": AXIS_STATE_NAMES.get(self.axis_state, "UNKNOWN"),
            "axis_state_hex": f"0x{self.axis_state:X}",
            "is_closed_loop": self.is_closed_loop,
            "is_in_error": self.is_in_error,
            "pos_estimate": round(self.pos_estimate, 4),
            "vel_estimate": round(self.vel_estimate, 4),
            "bus_voltage": round(self.bus_voltage, 2),
            "bus_current": round(self.bus_current, 2),
            "heartbeat_count": self.heartbeat_count,
        }


# ============================================================
# ODrive CAN 管理器
# ============================================================
class ODriveCanManager:
    """ODrive CAN 总线管理器."""

    def __init__(self, can_channel: str = 'can0', bitrate: int = 250000,
                 node_ids: list = None, can_interface: str = 'socketcan'):
        self.can_channel = can_channel
        self.bitrate = bitrate
        self.can_interface = can_interface
        self.node_ids = node_ids or [0, 1, 2, 3]
        self.devices: Dict[int, ODriveDevice] = {
            nid: ODriveDevice(nid) for nid in self.node_ids
        }
        self.bus = None
        self.running = False
        self.control_mode = CONTROL_MODE_VELOCITY
        self.lock = threading.Lock()

        # 速度控制参数
        self.wheel_radius = 0.085  # m
        self.track_width = 0.40    # m
        self.max_linear_vel = 5.58  # m/s
        self.max_angular_vel = 68.07  # rad/s

    def connect(self) -> bool:
        """连接 CAN 总线."""
        try:
            self.bus = can.interface.Bus(
                interface=self.can_interface,
                channel=self.can_channel,
                bitrate=self.bitrate,
            )
            print(
                f"[OK] CAN 总线连接: {self.can_interface}/{self.can_channel} "
                f"@ {self.bitrate} bps"
            )
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

    def scan_devices(self) -> list:
        """扫描 CAN 总线上的 ODrive 设备."""
        print("扫描 CAN 总线上的 ODrive 设备...")
        found = []

        # 监听 5 秒的心跳消息
        start_time = time.time()
        while time.time() - start_time < 5.0:
            msg = self.bus.recv(0.1)
            if msg:
                cmd_id, node_id = parse_can_id(msg.arbitration_id)
                if cmd_id == CMD_HEARTBEAT and node_id in self.devices:
                    self._parse_heartbeat(node_id, msg)
                    if node_id not in found:
                        found.append(node_id)

        print(f"发现 {len(found)} 个设备: {found}")
        return found

    def setup_device(self, node_id: int) -> bool:
        """完整设置设备（清错->设模式->进闭环）."""
        if node_id not in self.devices:
            return False

        self.send_cmd(node_id, CMD_CLEAR_ERRORS)
        time.sleep(0.05)
        self.send_cmd(node_id, CMD_SET_CONTROLLER_MODE,
                      struct.pack('<II', self.control_mode, 0x01))
        time.sleep(0.05)
        self.send_cmd(node_id, CMD_SET_AXIS_STATE,
                      struct.pack('<I', AXIS_STATE_CLOSED_LOOP_CONTROL))
        time.sleep(0.5)

        return self.devices[node_id].is_closed_loop

    def setup_all_devices(self) -> dict:
        """设置所有设备."""
        results = {}
        for nid in self.node_ids:
            results[nid] = self.setup_device(nid)
        return results

    def set_velocity(self, vx: float, wz: float) -> None:
        """设置左右轮速度 (m/s)."""
        # 差分运动学
        v_left = vx - wz * self.track_width / 2.0
        v_right = vx + wz * self.track_width / 2.0

        # 线速度 -> 角速度 (rad/s -> turn/s)
        w_left_turn = v_left / (2 * 3.14159 * self.wheel_radius)
        w_right_turn = v_right / (2 * 3.14159 * self.wheel_radius)

        # 发送给每个 ODrive
        with self.lock:
            for nid in self.node_ids:
                if nid < 2:  # 左轮
                    self.send_cmd(nid, CMD_SET_INPUT_VEL,
                                  struct.pack('<ff', w_left_turn, 0.0))
                else:  # 右轮
                    self.send_cmd(nid, CMD_SET_INPUT_VEL,
                                  struct.pack('<ff', w_right_turn, 0.0))

    def set_wheel_speeds(self, speeds: list) -> None:
        """直接设置四轮速度 (turn/s)."""
        with self.lock:
            for i, nid in enumerate(self.node_ids):
                if i < len(speeds):
                    self.send_cmd(nid, CMD_SET_INPUT_VEL,
                                  struct.pack('<ff', speeds[i], 0.0))

    def stop_all(self) -> None:
        """停止所有电机."""
        self.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])

    def estop_all(self) -> None:
        """急停所有电机."""
        with self.lock:
            for nid in self.node_ids:
                self.send_cmd(nid, CMD_ESTOP, b'')
                self.send_cmd(nid, CMD_SET_AXIS_STATE,
                              struct.pack('<I', AXIS_STATE_IDLE))

    def _parse_heartbeat(self, node_id: int, msg) -> None:
        """解析心跳消息."""
        if node_id not in self.devices:
            self.devices[node_id] = ODriveDevice(node_id)
        dev = self.devices[node_id]

        dev.last_heartbeat = time.time()
        dev.is_online = True
        dev.heartbeat_count += 1

        if len(msg.data) >= 4:
            dev.axis_state = struct.unpack('<I', msg.data[0:4])[0]
        if len(msg.data) >= 8:
            err_flags = struct.unpack('<I', msg.data[4:8])[0]
            dev.axis_error = (err_flags >> 16) & 0xFFFF
            dev.motor_error = err_flags & 0xFFFF

    def _parse_encoder(self, node_id: int, msg) -> None:
        """解析编码器估计消息."""
        if node_id not in self.devices:
            return
        dev = self.devices[node_id]
        if len(msg.data) >= 8:
            dev.pos_estimate, dev.vel_estimate = struct.unpack('<ff', msg.data[0:8])

    def _parse_bus_voltage(self, node_id: int, msg) -> None:
        """解析总线电压/电流消息."""
        if node_id not in self.devices:
            return
        dev = self.devices[node_id]
        if len(msg.data) >= 8:
            dev.bus_voltage, dev.bus_current = struct.unpack('<ff', msg.data[0:8])

    def receive_loop(self) -> None:
        """接收 CAN 消息循环."""
        print("开始接收 CAN 消息...")
        while self.running:
            msg = self.bus.recv(0.01)
            if msg:
                cmd_id, node_id = parse_can_id(msg.arbitration_id)
                if cmd_id == CMD_HEARTBEAT:
                    self._parse_heartbeat(node_id, msg)
                elif cmd_id == CMD_GET_ENCODER_ESTIMATES:
                    self._parse_encoder(node_id, msg)
                elif cmd_id == CMD_GET_BUS_VOLTAGE_CURRENT:
                    self._parse_bus_voltage(node_id, msg)

    def get_all_devices_status(self) -> dict:
        """获取所有设备状态，并将超时的设备标记为离线。"""
        now = time.time()
        for dev in self.devices.values():
            if dev.last_heartbeat == 0 or now - dev.last_heartbeat > 1.0:
                dev.is_online = False
        return {nid: dev.to_dict() for nid, dev in self.devices.items()}


# ============================================================
# Flask Web 应用
# ============================================================
app = Flask(__name__)

# 全局 CAN 管理器
can_manager: Optional['ODriveCanManager'] = None
receive_thread = None


def init_can_manager(can_interface: str, can_channel: str,
                     bitrate: int, node_ids: list) -> bool:
    """初始化 CAN 管理器。支持 Linux SocketCAN 与 Windows PCAN。"""
    global can_manager, receive_thread

    can_manager = ODriveCanManager(
        can_interface=can_interface,
        can_channel=can_channel,
        bitrate=bitrate,
        node_ids=node_ids,
    )

    if not can_manager.connect():
        print(
            f"[ERROR] 无法连接 CAN 总线 "
            f"{can_interface}/{can_channel}，请检查适配器驱动和通道配置"
        )
        if can_interface == 'socketcan':
            print(f"执行: sudo ip link set {can_channel} up type can bitrate {bitrate}")
        elif can_interface == 'pcan':
            print("Windows PCAN 需要安装 PEAK Device Driver Setup（含 PCAN-Basic）")
        return False

    can_manager.running = True
    receive_thread = threading.Thread(target=can_manager.receive_loop, daemon=True)
    receive_thread.start()

    return True


@app.route('/')
def index():
    """主页."""
    return render_template('index.html')


@app.route('/api/status')
def get_status():
    """获取所有设备状态."""
    if not can_manager:
        return jsonify({"error": "CAN 管理器未初始化"})
    return jsonify(can_manager.get_all_devices_status())


@app.route('/api/scan', methods=['POST'])
def scan_devices():
    """扫描设备."""
    if not can_manager:
        return jsonify({"error": "CAN 管理器未初始化"}), 500
    try:
        found = can_manager.scan_devices()
        return jsonify({"found": found})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/setup', methods=['POST'])
def setup_devices():
    """设置所有设备."""
    if not can_manager:
        return jsonify({"error": "CAN 管理器未初始化"}), 500
    try:
        results = can_manager.setup_all_devices()
        return jsonify({"results": results})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/velocity', methods=['POST'])
def set_velocity():
    """设置速度."""
    if not can_manager:
        return jsonify({"error": "CAN 管理器未初始化"}), 500
    try:
        data = request.json
        vx = float(data.get('vx', 0))
        wz = float(data.get('wz', 0))
        can_manager.set_velocity(vx, wz)
        return jsonify({"status": "ok", "vx": vx, "wz": wz})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/stop', methods=['POST'])
def stop_all():
    """停止所有电机."""
    if not can_manager:
        return jsonify({"error": "CAN 管理器未初始化"}), 500
    can_manager.stop_all()
    return jsonify({"status": "stopped"})


@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """急停."""
    if not can_manager:
        return jsonify({"error": "CAN 管理器未初始化"}), 500
    can_manager.estop_all()
    return jsonify({"status": "estop"})


@app.route('/api/mode', methods=['POST'])
def set_mode():
    """设置控制模式."""
    if not can_manager:
        return jsonify({"error": "CAN 管理器未初始化"}), 500
    try:
        data = request.json
        mode = data.get('mode', 'velocity')
        if mode == 'velocity':
            can_manager.control_mode = CONTROL_MODE_VELOCITY
        elif mode == 'position':
            can_manager.control_mode = CONTROL_MODE_POSITION
        return jsonify({"status": "ok", "mode": mode})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


def generate_status_stream():
    """实时状态流（Server-Sent Events）."""
    if not can_manager:
        return
    while True:
        status = can_manager.get_all_devices_status()
        yield "data: " + str(status).replace("'", '"') + "\n"
        time.sleep(0.1)


@app.route('/api/stream')
def stream():
    """实时状态流."""
    return Response(generate_status_stream(), mimetype='text/event-stream')


# ============================================================
# 主函数
# ============================================================
def main():
    """主函数."""
    import argparse

    parser = argparse.ArgumentParser(description='ODrive Web 控制服务器')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址')
    parser.add_argument('--port', '-p', type=int, default=5000, help='端口号')
    parser.add_argument('--debug', action='store_true', help='调试模式')
    default_interface = 'pcan' if platform.system() == 'Windows' else 'socketcan'
    default_channel = 'PCAN_USBBUS1' if platform.system() == 'Windows' else 'can0'
    parser.add_argument('--interface', default=default_interface,
                        help='python-can 接口（Windows 默认 pcan，Linux 默认 socketcan）')
    parser.add_argument('--channel', default=default_channel,
                        help='CAN 通道（PCAN 默认 PCAN_USBBUS1）')
    parser.add_argument('--bitrate', type=int, default=250000,
                        help='CAN 波特率（默认 250000）')
    parser.add_argument('--node-ids', type=int, nargs='+', default=[0, 1, 2, 3],
                        help='ODrive 节点 ID（默认 0 1 2 3）')

    args = parser.parse_args()

    print("=" * 60)
    print("  ODrive 小车 Web 控制服务器")
    print("=" * 60)
    print()

    print(f"运行平台: {platform.system()}")
    print(f"CAN 配置: {args.interface}/{args.channel} @ {args.bitrate} bps")

    # 初始化 CAN 管理器
    if not init_can_manager(args.interface, args.channel,
                            args.bitrate, args.node_ids):
        sys.exit(1)

    print()
    print("启动 Web 服务器...")
    print(f"访问地址: http://localhost:{args.port}")
    print(f"局域网访问: http://<Jetson-IP>:{args.port}")
    print()
    print("按 Ctrl+C 停止")
    print()

    try:
        app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)
    except KeyboardInterrupt:
        print("\n服务器已停止")
    finally:
        if can_manager:
            can_manager.stop_all()
            can_manager.estop_all()
            can_manager.running = False
            can_manager.disconnect()


if __name__ == '__main__':
    main()