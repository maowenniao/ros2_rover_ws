#!/usr/bin/env python3
"""
cmdvel_teleop_test.py - 简单手动测试 /cmd_vel 与 PX4 Setpoint 链路

用法：
    # 终端 1: 启动全部节点
    ros2 launch rover_bringup full_system.launch.py

    # 终端 2: 启动 teleop_twist_keyboard
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

    # 终端 3: 运行本脚本观察
    python3 cmdvel_teleop_test.py
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleStatus


class TeleopTestNode(Node):
    def __init__(self):
        super().__init__('cmdvel_teleop_test')
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Twist, '/cmd_vel',
                                 self.cb_cmdvel, qos_reliable)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                 self.cb_setpoint, qos_px4)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self.cb_status, qos_px4)

        self._cmdvel_count = 0
        self._setpoint_count = 0

    def cb_cmdvel(self, msg):
        self._cmdvel_count += 1
        self.get_logger().info(
            f'/cmd_vel #{self._cmdvel_count}: vx={msg.linear.x:.2f}, '
            f'wz={msg.angular.z:.2f}', throttle_duration_sec=0.5,
        )

    def cb_setpoint(self, msg):
        self._setpoint_count += 1
        self.get_logger().info(
            f'/fmu/in/trajectory_setpoint #{self._setpoint_count}: '
            f'vel=({msg.velocity[0]:.2f}, {msg.velocity[1]:.2f}, {msg.velocity[2]:.2f})',
            throttle_duration_sec=1.0,
        )

    def cb_status(self, msg):
        self.get_logger().info(
            f'PX4: nav_state={msg.nav_state}, arming_state={msg.arming_state}',
            throttle_duration_sec=2.0,
        )


def main():
    rclpy.init()
    node = TeleopTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()