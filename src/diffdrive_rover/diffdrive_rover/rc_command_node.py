#!/usr/bin/env python3
"""遥控器经安全层进底盘，并在手动/自动之间切换。

输出默认是 /cmd_vel_nav，这样建图和导航里的安全层仍能限速。
自动模式下转发 /cmd_vel_auto（Nav2 或外部遥控）。手动模式下只用摇杆，
不再把导航速度送出去。失控、急停开关或 PX4 failsafe 发布 /estop。
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import FailsafeFlags, ManualControlSetpoint, ManualControlSwitches, VehicleStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from diffdrive_rover.rc_logic import (
    estop_requested,
    rc_link_lost,
    select_velocity,
    sticks_to_body_velocity,
    update_mode,
)
from rover_description.geometry import declare_geometry


def _px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class RcCommandNode(Node):
    def __init__(self) -> None:
        super().__init__('rc_command_node')
        declare_geometry(self)
        self.declare_parameter('command_topic', '/cmd_vel_nav')
        self.declare_parameter('auto_topic', '/cmd_vel_auto')
        self.declare_parameter('relay_auto', True)
        self.declare_parameter('default_mode', 'auto')
        self.declare_parameter('deadband', 0.06)
        self.declare_parameter('expo', 0.30)
        self.declare_parameter('forward_channel', 'pitch')
        self.declare_parameter('yaw_channel', 'yaw')
        self.declare_parameter('mode_channel', 'aux1')
        self.declare_parameter('mode_threshold', 0.30)
        self.declare_parameter('estop_channel', 'aux2')
        self.declare_parameter('estop_threshold', 0.50)
        self.declare_parameter('rc_timeout_sec', 0.50)
        self.declare_parameter('auto_timeout_sec', 0.50)
        self.declare_parameter('publish_hz', 20.0)

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.expo = float(self.get_parameter('expo').value)
        self.forward_channel = str(self.get_parameter('forward_channel').value)
        self.yaw_channel = str(self.get_parameter('yaw_channel').value)
        self.mode_channel = str(self.get_parameter('mode_channel').value)
        self.mode_threshold = float(self.get_parameter('mode_threshold').value)
        self.estop_channel = str(self.get_parameter('estop_channel').value)
        self.estop_threshold = float(self.get_parameter('estop_threshold').value)
        self.rc_timeout_sec = float(self.get_parameter('rc_timeout_sec').value)
        self.auto_timeout_sec = float(self.get_parameter('auto_timeout_sec').value)
        self.relay_auto = bool(self.get_parameter('relay_auto').value)
        self.mode = str(self.get_parameter('default_mode').value)
        if self.mode not in ('manual', 'auto'):
            self.mode = 'auto'

        self._valid = False
        self._seen_rc = False
        self._last_rc = None
        self._pitch = 0.0
        self._yaw = 0.0
        self._mode_switch = None
        self._estop_switch = None
        self._kill = False
        self._signal_lost = False
        self._px4_failsafe = False
        self._geofence = False
        self._battery_critical = False
        self._auto_vx = 0.0
        self._auto_wz = 0.0
        self._last_auto = None
        self._estop_latched = False
        self._last_reason = None

        command_topic = str(self.get_parameter('command_topic').value)
        self._cmd_pub = self.create_publisher(Twist, command_topic, 10)
        self._mode_pub = self.create_publisher(String, '/control_mode', 10)
        self._estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.create_subscription(
            ManualControlSetpoint, '/fmu/out/manual_control_setpoint', self._on_manual, _px4_qos())
        self.create_subscription(
            ManualControlSwitches, '/fmu/out/manual_control_switches', self._on_switches, _px4_qos())
        self.create_subscription(
            FailsafeFlags, '/fmu/out/failsafe_flags', self._on_failsafe, _px4_qos())
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._on_status, _px4_qos())
        if self.relay_auto:
            self.create_subscription(
                Twist, str(self.get_parameter('auto_topic').value), self._on_auto, 10)
        period = 1.0 / max(1.0, float(self.get_parameter('publish_hz').value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'遥控器输出 {command_topic}，默认模式 {self.mode}，'
            f'模式开关 {self.mode_channel}，急停开关 {self.estop_channel}'
        )

    def _channel(self, msg, name: str):
        if not hasattr(msg, name):
            return None
        value = float(getattr(msg, name))
        if not math.isfinite(value):
            return None
        return value

    def _on_manual(self, msg: ManualControlSetpoint) -> None:
        self._last_rc = self.get_clock().now()
        self._seen_rc = True
        self._valid = bool(msg.valid)
        pitch = self._channel(msg, self.forward_channel)
        yaw = self._channel(msg, self.yaw_channel)
        self._pitch = 0.0 if pitch is None else pitch
        self._yaw = 0.0 if yaw is None else yaw
        self._mode_switch = self._channel(msg, self.mode_channel)
        self._estop_switch = self._channel(msg, self.estop_channel)
        self.mode = update_mode(self.mode, self._mode_switch, self.mode_threshold)

    def _on_switches(self, msg: ManualControlSwitches) -> None:
        self._kill = msg.kill_switch == ManualControlSwitches.SWITCH_POS_ON

    def _on_failsafe(self, msg: FailsafeFlags) -> None:
        self._signal_lost = bool(msg.manual_control_signal_lost)
        self._geofence = bool(msg.geofence_breached)
        self._battery_critical = msg.battery_warning >= 2

    def _on_status(self, msg: VehicleStatus) -> None:
        self._px4_failsafe = bool(msg.failsafe)

    def _on_auto(self, msg: Twist) -> None:
        self._auto_vx = float(msg.linear.x)
        self._auto_wz = float(msg.angular.z)
        self._last_auto = self.get_clock().now()

    def _age(self, stamp) -> float | None:
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _tick(self) -> None:
        lost = rc_link_lost(
            self._valid, self._age(self._last_rc), self.rc_timeout_sec,
            self._signal_lost, self._seen_rc,
        )
        reason = estop_requested(
            lost, self._kill, self._estop_switch, self.estop_threshold,
            self._px4_failsafe, self._geofence, self._battery_critical,
        )
        if reason is not None:
            if not self._estop_latched or reason != self._last_reason:
                self.get_logger().error(f'遥控器急停：{reason}')
                self._estop_pub.publish(Bool(data=True))
            self._estop_latched = True
            self._last_reason = reason
            self._publish_twist(0.0, 0.0)
            self._publish_mode('estop')
            return
        if self._estop_latched:
            self.get_logger().warning('遥控急停条件解除。电机仍保持去使能，需要重新 arm。')
            self._estop_pub.publish(Bool(data=False))
            self._estop_latched = False
            self._last_reason = None
        manual_vx, manual_wz = sticks_to_body_velocity(
            self._pitch, self._yaw, self.deadband, self.expo,
            self.max_linear_vel, self.max_angular_vel,
        )
        vx, wz, source = select_velocity(
            self.mode, manual_vx, manual_wz, self._auto_vx, self._auto_wz,
            self._age(self._last_auto), self.auto_timeout_sec,
        )
        self._publish_twist(vx, wz)
        self._publish_mode(source)

    def _publish_twist(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self._cmd_pub.publish(msg)

    def _publish_mode(self, source: str) -> None:
        self._mode_pub.publish(String(data=f'{self.mode}:{source}'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RcCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
