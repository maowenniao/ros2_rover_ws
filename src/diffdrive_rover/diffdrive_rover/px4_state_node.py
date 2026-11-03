#!/usr/bin/env python3
"""把 PX4 的机体系角速度和航向转成 ROS IMU，并把 GPS 转成 NavSatFix。

角速度和姿态取自 /fmu/out/vehicle_odometry。GPS 取自 Pixhawk 上的接收机
/fmu/out/vehicle_gps_position（SensorGps）。没有定位时不发布 /gps/fix。
室内没有这条消息时，节点照常发 IMU 和诊断，不会等待。
"""
from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import BatteryStatus, FailsafeFlags, SensorGps, VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix

from diffdrive_rover.gps_logic import (
    enu_orientation_from_odometry,
    gps_diagnostic,
    orientation_covariance,
    sensor_gps_to_navsat,
)
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
        self.declare_parameter('gps_frame', 'gps_link')
        self.declare_parameter('gps_stale_sec', 2.0)
        self._min_period = 1.0 / max(1.0, float(self.get_parameter('publish_hz').value))
        self._warn_remaining = float(self.get_parameter('battery_warn_remaining').value)
        self._gps_frame = str(self.get_parameter('gps_frame').value)
        self._gps_stale_sec = float(self.get_parameter('gps_stale_sec').value)
        self._last_pub = None
        self._battery = None
        self._status = None
        self._failsafe = None
        self._gps_rx = None
        self._gps_fix_type = 0
        self._gps_sats = 0
        self._gps_hdop = math.nan
        self._gps_eph = math.nan
        self._gps_epv = math.nan
        self._warned_pose_frame = False
        self._imu_pub = self.create_publisher(Imu, '/px4/imu', 10)
        self._gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self._on_odom, _px4_qos())
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self._on_battery, _px4_qos())
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._on_status, _px4_qos())
        self.create_subscription(FailsafeFlags, '/fmu/out/failsafe_flags', self._on_failsafe, _px4_qos())
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self._on_gps, _px4_qos())
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info('PX4 状态桥已启动：/px4/imu、/gps/fix 与 /diagnostics')

    def _on_odom(self, msg: VehicleOdometry) -> None:
        now = self.get_clock().now()
        if self._last_pub is not None:
            if (now - self._last_pub).nanoseconds / 1e9 < self._min_period:
                return
        rates = msg.angular_velocity
        if not all(math.isfinite(float(v)) for v in rates):
            return
        wx, wy, wz = frd_angular_to_flu((rates[0], rates[1], rates[2]))
        pose_frame = int(msg.pose_frame)
        if pose_frame not in (0, 1) and not self._warned_pose_frame:
            self._warned_pose_frame = True
            self.get_logger().warn('vehicle_odometry 姿态不是 NED，先不把航向写进 /px4/imu')
        quat = enu_orientation_from_odometry(pose_frame, msg.q)
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'base_link'
        imu.angular_velocity.x = wx
        imu.angular_velocity.y = wy
        imu.angular_velocity.z = wz
        imu.angular_velocity_covariance[0] = 0.02
        imu.angular_velocity_covariance[4] = 0.02
        imu.angular_velocity_covariance[8] = 0.02
        imu.linear_acceleration_covariance[0] = -1.0
        orientation = orientation_covariance(quat is not None, msg.orientation_variance)
        if quat is None:
            imu.orientation.w = 1.0
        else:
            imu.orientation.x = quat[0]
            imu.orientation.y = quat[1]
            imu.orientation.z = quat[2]
            imu.orientation.w = quat[3]
        imu.orientation_covariance = list(orientation)
        self._imu_pub.publish(imu)
        self._last_pub = now

    def _on_gps(self, msg: SensorGps) -> None:
        now = self.get_clock().now()
        self._gps_rx = now
        self._gps_fix_type = int(msg.fix_type)
        self._gps_sats = int(msg.satellites_used)
        self._gps_hdop = float(msg.hdop)
        self._gps_eph = float(msg.eph)
        self._gps_epv = float(msg.epv)
        fields = sensor_gps_to_navsat(
            msg.latitude_deg,
            msg.longitude_deg,
            msg.altitude_ellipsoid_m,
            msg.fix_type,
            msg.eph,
            msg.epv,
            self._gps_frame,
        )
        if fields is None:
            return
        fix = NavSatFix()
        fix.header.stamp = now.to_msg()
        fix.header.frame_id = fields.frame_id
        fix.status.status = fields.status
        fix.status.service = fields.service
        fix.latitude = fields.latitude
        fix.longitude = fields.longitude
        fix.altitude = fields.altitude
        fix.position_covariance = list(fields.covariance)
        fix.position_covariance_type = fields.covariance_type
        self._gps_pub.publish(fix)

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
        array.status.append(self._gps_status())
        self._diag_pub.publish(array)

    def _gps_status(self) -> DiagnosticStatus:
        age = None
        if self._gps_rx is not None:
            age = (self.get_clock().now() - self._gps_rx).nanoseconds / 1e9
        report = gps_diagnostic(
            self._gps_rx is not None,
            age,
            self._gps_stale_sec,
            self._gps_fix_type,
            self._gps_sats,
            self._gps_hdop,
            self._gps_eph,
            self._gps_epv,
        )
        status = DiagnosticStatus()
        status.name = 'px4_gps'
        # 这套消息里 level 是单字节，必须用 DiagnosticStatus 的常量，不能塞普通整数。
        status.level = {
            0: DiagnosticStatus.OK,
            1: DiagnosticStatus.WARN,
            2: DiagnosticStatus.ERROR,
            3: DiagnosticStatus.STALE,
        }[report.level]
        status.message = report.message
        status.values = [KeyValue(key=key, value=value) for key, value in report.values]
        return status

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
