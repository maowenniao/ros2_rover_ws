#!/usr/bin/env python3
"""由 /wheel_state 积分差速里程计，发布 /wheel_odom 和 odom → base_link。"""
from __future__ import annotations

import math
from typing import List

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from odrive_can_bridge.kinematics import (
    body_from_wheels,
    diagonal_covariance,
    integrate_pose,
    vehicle_wheel_speeds,
)
from rover_description.geometry import declare_geometry


class WheelOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odometry_node')
        declare_geometry(self)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('min_dt_seconds', 0.001)
        self.declare_parameter('max_dt_seconds', 0.2)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.directions: List[float] = [float(v) for v in self.get_parameter('wheel_directions').value]
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.min_dt = float(self.get_parameter('min_dt_seconds').value)
        self.max_dt = float(self.get_parameter('max_dt_seconds').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.pose_covariance = diagonal_covariance([0.05, 0.05, 1.0e6, 1.0e6, 1.0e6, 0.1])
        self.twist_covariance = diagonal_covariance([0.02, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 0.05])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(JointState, '/wheel_state', self._on_joint, qos)
        self._pub = self.create_publisher(Odometry, '/wheel_odom', qos)
        self._tf = TransformBroadcaster(self) if self.publish_odom_tf else None
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = None
        self._last_pub = None
        self._vx = 0.0
        self._wz = 0.0
        self.get_logger().info(
            f'轮速里程计：radius={self.wheel_radius} m, track={self.track_width} m, '
            f'{self.frame_id} → {self.child_frame_id}'
        )

    def _on_joint(self, msg: JointState) -> None:
        if len(msg.velocity) < 4:
            return
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt < self.min_dt or dt > self.max_dt:
            return
        left, right = vehicle_wheel_speeds(msg.velocity, self.directions)
        self._vx, self._wz = body_from_wheels(left, right, self.wheel_radius, self.track_width)
        self._x, self._y, self._theta = integrate_pose(self._x, self._y, self._theta, self._vx, self._wz, dt)
        self._publish(now)

    def _publish(self, now) -> None:
        if self._last_pub is not None:
            if (now - self._last_pub).nanoseconds / 1e9 < 1.0 / self.publish_rate:
                return
        self._last_pub = now
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        odom.pose.covariance = self.pose_covariance
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._wz
        odom.twist.covariance = self.twist_covariance
        self._pub.publish(odom)
        if self._tf is None:
            return
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
