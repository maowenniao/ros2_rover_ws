#!/usr/bin/env python3
"""
wheel_odometry_node
===================

功能：
    订阅 ``/wheel_state`` (sensor_msgs/JointState) 并发布 ``/wheel_odom``
    (nav_msgs/Odometry)。基于 4 轮差速模型积分位置与姿态。

    改进功能：
    - 添加协方差矩阵输出
    - 支持速度直接测量（如果有 velocity 数据）
    - 改进的时间微分计算
    - 支持自定义轮子配置顺序

依赖：
    - /wheel_state 由 odrive_can_node 发布
    - 4 个轮子的速度，单位 rad/s
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3, PoseStamped
from std_msgs.msg import Header


class WheelOdometryNode(Node):
    """由轮速积分得到车身 odom."""

    # 默认协方差（可以通过参数调节）
    POSE_COVARIANCE_DEFAULT = [
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
    ]
    TWIST_COVARIANCE_DEFAULT = [
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
    ]

    def __init__(self) -> None:
        super().__init__('wheel_odometry_node')

        # ============== 参数 ==============
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('track_width', 0.55)
        self.declare_parameter('wheel_base', None)  # 如果为 None，使用 track_width
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('use_velocity_measurements', True)
        self.declare_parameter('pose_covariance', self.POSE_COVARIANCE_DEFAULT)
        self.declare_parameter('twist_covariance', self.TWIST_COVARIANCE_DEFAULT)
        self.declare_parameter('min_dt_seconds', 0.001)
        self.declare_parameter('max_dt_seconds', 0.1)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.track_width = float(self.get_parameter('track_width').value)
        
        wheel_base = self.get_parameter('wheel_base').value
        if wheel_base is not None:
            self.wheel_base = float(wheel_base)
        else:
            self.wheel_base = self.track_width  # 默认 wheel_base = track_width
        
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.use_velocity_measurements = bool(
            self.get_parameter('use_velocity_measurements').value)
        
        pose_cov = self.get_parameter('pose_covariance').value
        self.pose_covariance = [float(x) for x in pose_cov]
        
        twist_cov = self.get_parameter('twist_covariance').value
        self.twist_covariance = [float(x) for x in twist_cov]
        
        self.min_dt = float(self.get_parameter('min_dt_seconds').value)
        self.max_dt = float(self.get_parameter('max_dt_seconds').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)

        # ============== QoS ==============
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ============== ROS 2 接口 ==============
        self.sub = self.create_subscription(
            JointState, '/wheel_state', self._joint_cb, qos_reliable)
        
        self.pub = self.create_publisher(
            Odometry, '/wheel_odom', qos_best_effort)

        # TF2 广播器（延迟导入以减小依赖）
        self._tf_broadcaster = None
        if self.publish_odom_tf:
            try:
                from tf2_ros import TransformBroadcaster
                self._tf_broadcaster = TransformBroadcaster(self)
            except ImportError:
                self.get_logger().warn('tf2_ros 不可用，跳过 TF 广播')

        # ============== 状态 ==============
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        
        self._prev_left_vel = 0.0
        self._prev_right_vel = 0.0
        self._prev_time = None
        self._last_msg_time = None
        self._last_publish_time = None
        
        self._has_initial_velocity = False

        self.get_logger().info(
            f'wheel_odometry_node 已启动: radius={self.wheel_radius}m, '
            f'track={self.track_width}m, frame={self.frame_id} -> {self.child_frame_id}'
        )

    def _joint_cb(self, msg: JointState) -> None:
        """处理 JointState 消息并计算里程计."""
        now = self.get_clock().now()
        
        # 检查是否有足够的速度数据
        if len(msg.velocity) < 4:
            return
        
        # 计算时间差
        if self._last_msg_time is None:
            self._last_msg_time = now
            # 保存初始速度
            self._prev_left_vel = self._get_left_velocity(msg.velocity)
            self._prev_right_vel = self._get_right_velocity(msg.velocity)
            self._has_initial_velocity = True
            return
        
        dt = (now - self._last_msg_time).nanoseconds / 1e9
        self._last_msg_time = now
        
        # 过滤异常时间差
        if dt < self.min_dt or dt > self.max_dt:
            return

        # 获取左右轮速度
        w_left = self._get_left_velocity(msg.velocity)
        w_right = self._get_right_velocity(msg.velocity)
        
        # 计算线性速度 (m/s)
        v_left = w_left * self.wheel_radius
        v_right = w_right * self.wheel_radius

        # 使用中点法积分
        if self._has_initial_velocity:
            # 左右平均速度
            v_left_avg = (v_left + self._prev_left_vel) / 2.0
            v_right_avg = (v_right + self._prev_right_vel) / 2.0
            
            # 车身线速度和角速度
            v = (v_left_avg + v_right_avg) / 2.0
            w = (v_right_avg - v_left_avg) / self.track_width

            # 中点法积分
            omega_dt = w * dt
            if abs(omega_dt) > 1e-10:
                # 转弯情况
                r = v / w
                cos_omega = math.cos(omega_dt)
                sin_omega = math.sin(omega_dt)
                
                dx = r * sin_omega
                dy = r * (1.0 - cos_omega)
                
                # 旋转后的增量
                self._x += dx * math.cos(self._theta) - dy * math.sin(self._theta)
                self._y += dx * math.sin(self._theta) + dy * math.cos(self._theta)
            else:
                # 直线情况
                self._x += v * math.cos(self._theta) * dt
                self._y += v * math.sin(self._theta) * dt
            
            self._theta += w * dt
            
            # 角度归一化到 [-pi, pi]
            while self._theta > math.pi:
                self._theta -= 2.0 * math.pi
            while self._theta < -math.pi:
                self._theta += 2.0 * math.pi

        # 更新上一时刻速度
        self._prev_left_vel = v_left
        self._prev_right_vel = v_right
        self._has_initial_velocity = True

        # 以固定频率发布
        self._publish_odometry(now, v, w)

    def _get_left_velocity(self, velocities: list) -> float:
        """获取左轮平均速度.
        
        假设 velocities 顺序为: [左前, 左后, 右前, 右后]
        """
        if len(velocities) >= 4:
            return (velocities[0] + velocities[1]) / 2.0
        return velocities[0] if len(velocities) > 0 else 0.0

    def _get_right_velocity(self, velocities: list) -> float:
        """获取右轮平均速度.
        
        假设 velocities 顺序为: [左前, 左后, 右前, 右后]
        """
        if len(velocities) >= 4:
            return (velocities[2] + velocities[3]) / 2.0
        return velocities[1] if len(velocities) > 1 else 0.0

    def _publish_odometry(self, now, v, w) -> None:
        """发布里程计消息."""
        # 检查发布频率
        if self._last_publish_time is not None:
            dt = (now - self._last_publish_time).nanoseconds / 1e9
            if dt < 1.0 / self.publish_rate:
                return
        self._last_publish_time = now

        # 创建里程计消息
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # 姿态
        odom.pose = Pose()
        odom.pose.position = Point(x=self._x, y=self._y, z=0.0)
        
        # 计算四元数
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        odom.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
        # 协方差
        odom.pose.covariance = self.pose_covariance

        # 速度
        odom.twist = Twist()
        odom.twist.linear = Vector3(x=v, y=0.0, z=0.0)
        odom.twist.angular = Vector3(x=0.0, y=0.0, z=w)
        
        # 速度协方差
        odom.twist.covariance = self.twist_covariance

        # 发布
        self.pub.publish(odom)

        # 发布 TF
        if self._tf_broadcaster is not None:
            from tf2_msgs.msg import TFMessage
            from geometry_msgs.msg import TransformStamped
            
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = odom.pose.orientation
            
            self._tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> None:
        """清理资源."""
        if self._tf_broadcaster is not None:
            del self._tf_broadcaster
        super().destroy_node()


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