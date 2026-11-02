#!/usr/bin/env python3
"""把 PX4 EKF2 的机体系角速度转成 ROS IMU，并上报电池、解锁和 failsafe。

角速度取自已经在 uXRCE 默认列表里的 /fmu/out/vehicle_odometry，
不再订阅更高频的 vehicle_angular_velocity 或 D435i IMU。
"""
from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import BatteryStatus, FailsafeFlags, VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from diffdrive_rover.rc_logic import frd_angular_to_flu


def _px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class Px4StateNode(Node):
    def __init__(self) -> None:
        super().__init__('px4_state_node')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('battery_warn_remaining', 0.20)
        self._min_period = 1.0 / max(1.0, float(self.get_parameter('publish_hz').value))
        self._warn_remaining = float(self.get_parameter('battery_warn_remaining').value)
        self._last_pub = None
        self._battery = None
        self._status = None
        self._failsafe = None
        self._imu_pub = self.create_publisher(Imu, '/px4/imu', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self._on_odom, _px4_qos())
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self._on_battery, _px4_qos())
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._on_status, _px4_qos())
        self.create_subscription(FailsafeFlags, '/fmu/out/failsafe_flags', self._on_failsafe, _px4_qos())
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info('PX4 状态桥已启动：/px4/imu 与 /diagnostics')

    def _on_odom(self, msg: VehicleOdometry) -> None:
        now = self.get_clock().now()
        if self._last_pub is not None:
            if (now - self._last_pub).nanoseconds / 1e9 < self._min_period:
                return
        rates = msg.angular_velocity
        if not all(math.isfinite(float(v)) for v in rates):
            return
        wx, wy, wz = frd_angular_to_flu((rates[0], rates[1], rates[2]))
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'base_link'
        imu.angular_velocity.x = wx
        imu.angular_velocity.y = wy
        imu.angular_velocity.z = wz
        imu.angular_velocity_covariance[0] = 0.02
        imu.angular_velocity_covariance[4] = 0.02
        imu.angular_velocity_covariance[8] = 0.02
        self._imu_pub.publish(imu)
        self._last_pub = now

    def _on_battery(self, msg: BatteryStatus) -> None:
        self._battery = msg

    def _on_status(self, msg: VehicleStatus) -> None:
        self._status = msg

    def _on_failsafe(self, msg: FailsafeFlags) -> None:
        self._failsafe = msg

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(self._battery_status())
        array.status.append(self._vehicle_status())
        self._diag_pub.publish(array)

    def _battery_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = 'px4_battery'
        battery = self._battery
        if battery is None:
            status.level = DiagnosticStatus.STALE
            status.message = '还没有电池消息'
            return status
        remaining = float(battery.remaining)
        warning = int(battery.warning)
        if warning >= BatteryStatus.WARNING_CRITICAL or not battery.connected:
            status.level = DiagnosticStatus.ERROR
        elif warning >= BatteryStatus.WARNING_LOW or (0.0 <= remaining <= self._warn_remaining):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = f'{battery.voltage_v:.1f} V'
        status.values = [
            KeyValue(key='voltage_v', value=f'{battery.voltage_v:.2f}'),
            KeyValue(key='current_a', value=f'{battery.current_a:.2f}'),
            KeyValue(key='remaining', value=f'{remaining:.2f}'),
            KeyValue(key='warning', value=str(warning)),
        ]
        return status

    def _vehicle_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = 'px4_vehicle'
        vehicle = self._status
        flags = self._failsafe
        if vehicle is None:
            status.level = DiagnosticStatus.STALE
            status.message = '还没有飞控状态'
            return status
        armed = vehicle.arming_state == VehicleStatus.ARMING_STATE_ARMED
        problems = []
        if vehicle.failsafe:
            problems.append('failsafe')
        if flags is not None:
            if flags.manual_control_signal_lost:
                problems.append('rc_lost')
            if flags.gcs_connection_lost:
                problems.append('gcs_lost')
            if flags.geofence_breached:
                problems.append('geofence')
            if flags.battery_unhealthy:
                problems.append('battery_unhealthy')
        status.level = DiagnosticStatus.ERROR if problems else DiagnosticStatus.OK
        status.message = 'armed' if armed else 'disarmed'
        if problems:
            status.message += ' ' + ','.join(problems)
        status.values = [
            KeyValue(key='arming_state', value=str(int(vehicle.arming_state))),
            KeyValue(key='nav_state', value=str(int(vehicle.nav_state))),
            KeyValue(key='failsafe', value=str(bool(vehicle.failsafe))),
        ]
        return status


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4StateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
