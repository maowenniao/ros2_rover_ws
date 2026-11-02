#!/usr/bin/env python3
"""ODrive V3.6 CAN 桥。

订阅 /wheel_cmd（rad/s，顺序左前、左后、右前、右后），编码成 CANSimple
Set_Input_Vel。只在 /odrive_enable 为 true 且未急停时进入闭环。
/wheel_cmd 超过 cmd_timeout_sec 没有更新时改发 0，避免驱动器锁存旧速度。
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

from rover_description.geometry import declare_geometry
from odrive_can_bridge.can_protocol import (
    AXIS_STATE_CLOSED_LOOP,
    AXIS_STATE_IDLE,
    CMD_CLEAR_ERRORS,
    CMD_ESTOP,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_HEARTBEAT,
    CONTROL_MODE_VELOCITY,
    INPUT_MODE_PASSTHROUGH,
    decode_bus_voltage_current,
    decode_encoder_estimates,
    decode_heartbeat,
    encode_empty,
    encode_set_axis_state,
    encode_set_controller_mode,
    encode_set_input_vel,
    encode_set_limits,
    parse_can_id,
    rad_s_to_turns,
    select_velocity_command,
    turns_to_rad,
)


def latched_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class _Axis:
    def __init__(self) -> None:
        self.last_heartbeat = 0.0
        self.axis_error = 0
        self.axis_state = 0
        self.motor_error = False
        self.encoder_error = False
        self.controller_error = False
        self.pos_turns = 0.0
        self.vel_turns = 0.0
        self.bus_voltage = 0.0
        self.bus_current = 0.0
        self.seen = False

    @property
    def online(self) -> bool:
        return self.seen and (time.monotonic() - self.last_heartbeat) < 1.0

    @property
    def closed_loop(self) -> bool:
        return self.axis_state == AXIS_STATE_CLOSED_LOOP


class ODriveCanNode(Node):
    def __init__(self) -> None:
        super().__init__('odrive_can_node')
        self.declare_parameter('can_interface', 'socketcan')
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('node_ids', [0, 1, 2, 3])
        self.declare_parameter('wheel_cmd_order', [0, 1, 2, 3])
        self.declare_parameter('cmd_timeout_sec', 0.2)
        self.declare_parameter('cmd_send_rate_hz', 50.0)
        self.declare_parameter('encoder_poll_hz', 20.0)
        self.declare_parameter('current_lim', 10.0)
        self.declare_parameter('heartbeat_timeout_sec', 1.0)
        declare_geometry(self)

        self.can_interface = str(self.get_parameter('can_interface').value)
        self.can_channel = str(self.get_parameter('can_channel').value)
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.node_ids = [int(v) for v in self.get_parameter('node_ids').value]
        self.wheel_cmd_order = [int(v) for v in self.get_parameter('wheel_cmd_order').value]
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.current_lim = float(self.get_parameter('current_lim').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.heartbeat_timeout_sec = float(self.get_parameter('heartbeat_timeout_sec').value)
        cmd_hz = float(self.get_parameter('cmd_send_rate_hz').value)
        enc_hz = float(self.get_parameter('encoder_poll_hz').value)

        try:
            import can
        except ImportError as exc:
            raise RuntimeError('缺少 python-can，请安装 python3-can') from exc

        self._bus = can.interface.Bus(
            interface=self.can_interface,
            channel=self.can_channel,
            bitrate=self.can_bitrate,
        )
        self._axes: Dict[int, _Axis] = {nid: _Axis() for nid in self.node_ids}
        self._latest: Optional[List[float]] = None
        self._latest_time: Optional[float] = None
        self._enabled = False
        self._estopped = False
        self._last_setup = 0.0

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Float32MultiArray, '/wheel_cmd', self._on_wheel_cmd, reliable)
        self.create_subscription(Bool, '/odrive_enable', self._on_enable, latched_qos())
        self.create_subscription(Bool, '/estop', self._on_estop, latched_qos())
        self._state_pub = self.create_publisher(JointState, '/wheel_state', reliable)
        self._diag_pub = self.create_publisher(DiagnosticStatus, '/odrive_diagnostic', 10)
        self._online_pub = self.create_publisher(Bool, '/odrive_online', 10)

        self.create_timer(1.0 / cmd_hz, self._send_commands)
        self.create_timer(1.0 / max(enc_hz, 1.0), self._request_feedback)
        self.create_timer(0.02, self._poll_bus)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f'ODrive CAN 已连接 {self.can_interface}/{self.can_channel}，'
            f'等待 /odrive_enable 后才进入闭环；指令超时 {self.cmd_timeout_sec:.2f}s'
        )

    def _on_wheel_cmd(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < len(self.wheel_cmd_order):
            self.get_logger().warn(
                f'/wheel_cmd 长度 {len(msg.data)} < {len(self.wheel_cmd_order)}',
                throttle_duration_sec=1.0,
            )
            return
        self._latest = [float(v) for v in msg.data]
        self._latest_time = time.monotonic()

    def _on_enable(self, msg: Bool) -> None:
        if self._estopped and msg.data:
            self.get_logger().warn('急停锁存中，忽略使能。先确认 /estop 为 false 再解锁。')
            return
        self._enabled = bool(msg.data)
        if not self._enabled:
            self._idle_all()
            self.get_logger().info('电机使能关闭，轴回到 IDLE')
        else:
            self.get_logger().info('收到使能，准备进入闭环')

    def _on_estop(self, msg: Bool) -> None:
        if not msg.data:
            # 只解除锁存，不自动回到闭环。需要再次收到 /odrive_enable。
            self._estopped = False
            return
        if not self._estopped:
            self.get_logger().error('收到 /estop，发送 ODrive Estop 并退出闭环')
        self._estopped = True
        self._enabled = False
        self._latest = [0.0] * len(self.wheel_cmd_order)
        self._estop_all()

    def _send_frame(self, arbitration_id: int, data: bytes) -> None:
        import can
        self._bus.send(can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        ))

    def _estop_all(self) -> None:
        for nid in self.node_ids:
            arb, data = encode_empty(nid, CMD_ESTOP)
            self._send_frame(arb, data)
            arb, data = encode_set_axis_state(nid, AXIS_STATE_IDLE)
            self._send_frame(arb, data)

    def _idle_all(self) -> None:
        for nid in self.node_ids:
            arb, data = encode_set_input_vel(nid, 0.0, 0.0)
            self._send_frame(arb, data)
            arb, data = encode_set_axis_state(nid, AXIS_STATE_IDLE)
            self._send_frame(arb, data)

    def _enter_closed_loop(self, node_id: int) -> None:
        vel_limit_turns = rad_s_to_turns(self.max_wheel_speed)
        for arb, data in (
            encode_empty(node_id, CMD_CLEAR_ERRORS),
            encode_set_controller_mode(node_id, CONTROL_MODE_VELOCITY, INPUT_MODE_PASSTHROUGH),
            encode_set_limits(node_id, vel_limit_turns, self.current_lim),
            encode_set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP),
        ):
            self._send_frame(arb, data)

    def _all_closed_loop(self) -> bool:
        return all(self._axes[nid].closed_loop for nid in self.node_ids)

    def _send_commands(self) -> None:
        now = time.monotonic()
        if self._enabled and not self._estopped and now - self._last_setup >= 1.0:
            requested = False
            for nid in self.node_ids:
                axis = self._axes[nid]
                if axis.seen and not axis.closed_loop:
                    self._enter_closed_loop(nid)
                    requested = True
            if requested:
                self._last_setup = now
        age = None if self._latest_time is None else time.monotonic() - self._latest_time
        command = select_velocity_command(
            self._latest,
            age,
            self.cmd_timeout_sec,
            self._enabled,
            self._estopped,
            self._all_closed_loop(),
            len(self.wheel_cmd_order),
        )
        if command is None:
            return
        for index, node_id in enumerate(self.wheel_cmd_order):
            turns = rad_s_to_turns(command[index])
            arb, data = encode_set_input_vel(node_id, turns, 0.0)
            self._send_frame(arb, data)

    def _request_feedback(self) -> None:
        for nid in self.node_ids:
            arb, data = encode_empty(nid, CMD_GET_ENCODER_ESTIMATES)
            self._send_frame(arb, data)
            arb, data = encode_empty(nid, CMD_GET_BUS_VOLTAGE_CURRENT)
            self._send_frame(arb, data)

    def _poll_bus(self) -> None:
        try:
            while True:
                msg = self._bus.recv(timeout=0.0)
                if msg is None:
                    break
                self._parse(msg)
        except Exception as exc:  # noqa: BLE001 - 总线抖动不应杀掉节点
            self.get_logger().warn(f'CAN 读取异常: {exc}', throttle_duration_sec=1.0)
        self._publish_joint_state()
        online = Bool()
        online.data = bool(self.node_ids) and all(
            self._axes[nid].seen and (time.monotonic() - self._axes[nid].last_heartbeat) <= self.heartbeat_timeout_sec
            for nid in self.node_ids
        )
        self._online_pub.publish(online)

    def _parse(self, msg) -> None:
        cmd, node_id = parse_can_id(msg.arbitration_id)
        if node_id not in self._axes:
            return
        axis = self._axes[node_id]
        data = bytes(msg.data)
        if cmd == CMD_HEARTBEAT and len(data) >= 8:
            hb = decode_heartbeat(data)
            axis.axis_error = hb.axis_error
            axis.axis_state = hb.axis_state
            axis.motor_error = hb.motor_error
            axis.encoder_error = hb.encoder_error
            axis.controller_error = hb.controller_error
            axis.last_heartbeat = time.monotonic()
            axis.seen = True
        elif cmd == CMD_GET_ENCODER_ESTIMATES and len(data) >= 8:
            axis.pos_turns, axis.vel_turns = decode_encoder_estimates(data)
        elif cmd == CMD_GET_BUS_VOLTAGE_CURRENT and len(data) >= 8:
            axis.bus_voltage, axis.bus_current = decode_bus_voltage_current(data)

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'wheel_{nid}' for nid in self.node_ids]
        msg.position = [turns_to_rad(self._axes[nid].pos_turns) for nid in self.node_ids]
        msg.velocity = [turns_to_rad(self._axes[nid].vel_turns) for nid in self.node_ids]
        msg.effort = [0.0 for _ in self.node_ids]
        self._state_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        for nid in self.node_ids:
            axis = self._axes[nid]
            status = DiagnosticStatus()
            status.name = f'ODrive {nid}'
            fresh = axis.seen and (time.monotonic() - axis.last_heartbeat) <= self.heartbeat_timeout_sec
            if not fresh:
                status.level = DiagnosticStatus.ERROR
                status.message = 'OFFLINE'
            elif axis.axis_error or axis.motor_error or axis.encoder_error or axis.controller_error:
                status.level = DiagnosticStatus.ERROR
                status.message = f'FAULT state={axis.axis_state} axis_error=0x{axis.axis_error:X}'
            elif not axis.closed_loop:
                status.level = DiagnosticStatus.WARN
                status.message = f'state={axis.axis_state}'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'CLOSED_LOOP'
            status.values = [
                KeyValue(key='online', value='yes' if fresh else 'no'),
                KeyValue(key='axis_state', value=str(axis.axis_state)),
                KeyValue(key='axis_error', value=f'0x{axis.axis_error:X}'),
                KeyValue(key='vel_rad_s', value=f'{turns_to_rad(axis.vel_turns):.3f}'),
                KeyValue(key='bus_voltage', value=f'{axis.bus_voltage:.2f}'),
            ]
            self._diag_pub.publish(status)

    def destroy_node(self):
        try:
            self._estop_all()
        except Exception:
            pass
        try:
            self._bus.shutdown()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ODriveCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
