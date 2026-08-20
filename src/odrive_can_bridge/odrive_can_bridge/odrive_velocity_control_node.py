#!/usr/bin/env python3
"""
odrive_velocity_control_node
==============================

功能：
    简化版速度控制节点，直接接收线速度 + 角速度指令，
    计算左右轮速度并发布到 /wheel_cmd 供 ODrive CAN 桥接节点使用。

    与传统 cmdvel_to_setpoint_node 的区别：
    - 不经过 PX4，直接控制 ODrive
    - 支持速度模式和位置模式
    - 内置安全限幅和超时保护

订阅：
    /cmd_vel (geometry_msgs/Twist) - 速度指令

发布：
    /wheel_cmd (std_msgs/Float32MultiArray) - 四轮速度指令
          顺序：[左前, 左后, 右前, 右后]，单位 rad/s
"""
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class ODriveVelocityControlNode(Node):
    """ODrive 速度控制节点。"""

    def __init__(self) -> None:
        super().__init__('odrive_velocity_control_node')

        # ============== 参数 ==============
        self.declare_parameter('wheel_radius', 0.085)
        self.declare_parameter('track_width', 0.40)
        self.declare_parameter('max_linear_vel', 5.58)
        self.declare_parameter('max_angular_vel', 68.07)
        self.declare_parameter('cmdvel_timeout', 0.5)
        self.declare_parameter('enable_safety_clamp', True)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.cmdvel_timeout = float(self.get_parameter('cmdvel_timeout').value)
        self.enable_safety_clamp = bool(
            self.get_parameter('enable_safety_clamp').value)

        # ============== QoS ==============
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ============== 订阅 ==============
        self.cmdvel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmdvel_cb,
            qos_reliable,
        )

        # ============== 发布 ==============
        self.wheel_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_cmd',
            10,
        )

        # ============== 内部状态 ==============
        self._last_cmdvel_time = self.get_clock().now()
        self._current_vx = 0.0   # 前进方向速度 (m/s)
        self._current_wz = 0.0   # 角速度 (rad/s)

        # 创建超时检查定时器 (50Hz)
        self.create_timer(0.02, self._timeout_check)

        self.get_logger().info(
            f'odrive_velocity_control_node 已启动：'
            f'wheel_radius={self.wheel_radius}m, '
            f'track_width={self.track_width}m, '
            f'max_linear_vel={self.max_linear_vel}m/s, '
            f'max_angular_vel={self.max_angular_vel}rad/s'
        )

    # ------------------------------------------------------------------
    # /cmd_vel 回调
    # ------------------------------------------------------------------
    def _cmdvel_cb(self, msg: Twist) -> None:
        """/cmd_vel 回调，提取线速度和角速度."""
        self._current_vx = float(msg.linear.x)
        self._current_wz = float(msg.angular.z)
        self._last_cmdvel_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # 对外命令（可由其他节点调用）
    # ------------------------------------------------------------------
    def set_velocity(self, vx: float, wz: float) -> None:
        """直接设置目标速度.
        
        Args:
            vx: 前进方向速度 (m/s)
            wz: 角速度 (rad/s)
        """
        self._current_vx = vx
        self._current_wz = wz
        self._last_cmdvel_time = self.get_clock().now()
        self._publish_wheel_cmd()

    def stop(self) -> None:
        """停止移动."""
        self._current_vx = 0.0
        self._current_wz = 0.0
        self._publish_wheel_cmd()

    # ------------------------------------------------------------------
    # 周期性的超时检查（使用定时器）
    # ------------------------------------------------------------------
    def __init_timer(self) -> None:
        """创建超时检查定时器."""
        self.create_timer(0.02, self._timeout_check)  # 50Hz

    def _timeout_check(self) -> None:
        """检查 cmd_vel 超时，超时则清零速度."""
        now = self.get_clock().now()
        dt = (now - self._last_cmdvel_time).nanoseconds / 1e9

        if dt > self.cmdvel_timeout:
            if self._current_vx != 0.0 or self._current_wz != 0.0:
                self.get_logger().debug('cmd_vel 超时，停止发送速度指令')
                self._current_vx = 0.0
                self._current_wz = 0.0

        self._publish_wheel_cmd()

    def _publish_wheel_cmd(self) -> None:
        """解算四轮速度并发布."""
        # 安全限幅
        if self.enable_safety_clamp:
            self._current_vx = self._clamp(
                self._current_vx,
                -self.max_linear_vel,
                self.max_linear_vel,
            )
            self._current_wz = self._clamp(
                self._current_wz,
                -self.max_angular_vel,
                self.max_angular_vel,
            )

        # 差分运动学：左右轮速度 (m/s)
        v_left = self._current_vx - self._current_wz * self.track_width / 2.0
        v_right = self._current_vx + self._current_wz * self.track_width / 2.0

        # 线速度 -> 角速度 (rad/s)
        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius

        # 发布：[左前, 左后, 右前, 右后]
        wc_msg = Float32MultiArray()
        wc_msg.data = [w_left, w_left, w_right, w_right]
        self.wheel_cmd_pub.publish(wc_msg)

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        """限幅."""
        return max(lo, min(hi, v))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ODriveVelocityControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()