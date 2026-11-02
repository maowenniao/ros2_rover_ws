#!/usr/bin/env python3
"""
odometry_bridge_node
====================

功能：
    订阅 PX4 发布的 ``/fmu/out/vehicle_odometry`` (px4_msgs/VehicleOdometry)，
    转换为 ROS 2 标准 ``nav_msgs/Odometry``，发布到 ``/odom``。

    同时将机体坐标系下的速度发布为 ``/odom/twist`` (geometry_msgs/TwistStamped)。

依赖：
    px4_msgs >= 1.14
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Twist, \
                              Point, Quaternion, Vector3
from std_msgs.msg import Header


class OdometryBridgeNode(Node):
    """PX4 VehicleOdometry → ROS 2 nav_msgs/Odometry 桥接节点。"""

    def __init__(self) -> None:
        super().__init__('odometry_bridge_node')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        # PX4 用 BEST_EFFORT
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._odom_cb,
            qos_px4,
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_reliable)

        self._tf_broadcaster = None
        publish_tf = self.get_parameter('publish_tf').value
        if isinstance(publish_tf, str):
            publish_tf = publish_tf.lower() in ('1', 'true', 'yes', 'on')
        if publish_tf:
            from tf2_ros import TransformBroadcaster  # type: ignore
            self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('odometry_bridge_node 已启动')

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        # PX4 VehicleOdometry 字段：
        #   timestamp, pose_frame, velocity_frame, position[3], q[4],
        #   velocity[3], angular_velocity[3]
        # pose_frame=1 (LOCAL_NED), velocity_frame=1
        # 默认 PX4 用 ENU 取决于参数；本节点默认 PX4 输出为 NED

        # NED → ENU 转换（ROS 习惯 ENU）
        #   position_ned = [x_n, y_e, z_d]  →  ros_enu = [y_n, x_e, -z_d]
        # 注意：PX4 ROS2 默认 local_frame 是 FRD（机体），所以这里要视情况而定
        # 我们假设 VehicleOdometry 输出为 NED 系（pose_frame=1）下：
        if msg.pose_frame == VehicleOdometry.POSE_FRAME_NED:
            x = float(msg.position[1])    # NED_y → ROS_x
            y = float(msg.position[0])    # NED_x → ROS_y
            z = -float(msg.position[2])   # NED_z(down) → ROS_z(up)
            # 四元数 NED → ENU
            qx, qy, qz, qw = self._quat_ned_to_enu(
                float(msg.q[0]), float(msg.q[1]),
                float(msg.q[2]), float(msg.q[3]),
            )
            vx = float(msg.velocity[1])
            vy = float(msg.velocity[0])
            vz = -float(msg.velocity[2])
            wx = float(msg.angular_velocity[1])
            wy = float(msg.angular_velocity[0])
            wz = -float(msg.angular_velocity[2])
        else:
            # 假定 FRD 局部系：position 已经是 NED，姿态是机体 FRD
            x = float(msg.position[1])
            y = float(msg.position[0])
            z = -float(msg.position[2])
            qx, qy, qz, qw = self._quat_ned_to_enu(
                float(msg.q[0]), float(msg.q[1]),
                float(msg.q[2]), float(msg.q[3]),
            )
            vx = float(msg.velocity[1])
            vy = float(msg.velocity[0])
            vz = -float(msg.velocity[2])
            wx = float(msg.angular_velocity[1])
            wy = float(msg.angular_velocity[0])
            wz = -float(msg.angular_velocity[2])

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose = PoseWithCovariance()
        odom.pose.pose = Pose()
        odom.pose.pose.position = Point(x=x, y=y, z=z)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom.pose.covariance = [0.0] * 36
        # 占位协方差
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 0.05

        odom.twist = TwistWithCovariance()
        odom.twist.twist = Twist()
        odom.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)
        odom.twist.twist.angular = Vector3(x=wx, y=wy, z=wz)
        odom.twist.covariance = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            odom.twist.covariance[i] = 0.02

        self.odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            from geometry_msgs.msg import TransformStamped
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = odom.child_frame_id
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    @staticmethod
    def _quat_ned_to_enu(qx: float, qy: float, qz: float, qw: float):
        """NED 系四元数 → ENU 系四元数。"""
        # 等价于绕 x 翻转 180° 后再绕 z 翻转 180°（NED→ENU）
        # 简化做法：将姿态看作「heading=yaw」时只需要翻转 yaw 符号并补偿
        # 这里采用轴角翻转公式：
        half = np.array([qx, qy, qz, qw]) * 0.5
        # 翻转：-180° yaw，180° roll
        cy = np.cos(half[2]); sy = np.sin(half[2])  # yaw/2
        cp = np.cos(half[0]); sp = np.sin(half[0])  # roll/2
        cr = np.cos(half[1]); sr = np.sin(half[1])  # pitch/2
        # 翻转 z 轴 (yaw + π) & x 轴 (roll + π)
        cy2 = np.cos(np.pi / 2.0); sy2 = np.sin(np.pi / 2.0)
        cp2 = np.cos(np.pi / 2.0); sp2 = np.sin(np.pi / 2.0)
        # q_yaw_pi = (0, 0, sin(pi/2), cos(pi/2)) = (0,0,1,0)
        # q_roll_pi = (sin(pi/2), 0, 0, cos(pi/2)) = (1,0,0,0)
        # 合成: q_out = q_yaw_pi * q_ned * q_roll_pi
        def qmul(a, b):
            return np.array([
                a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
                a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
                a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
                a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2],
            ])
        qy_pi = np.array([0.0, 0.0, np.sin(np.pi / 2.0), np.cos(np.pi / 2.0)])
        qr_pi = np.array([np.sin(np.pi / 2.0), 0.0, 0.0, np.cos(np.pi / 2.0)])
        q_ned = np.array([qx, qy, qz, qw])
        q_out = qmul(qy_pi, qmul(q_ned, qr_pi))
        return float(q_out[0]), float(q_out[1]), float(q_out[2]), float(q_out[3])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()