#!/usr/bin/env python3
"""
arm_rover - 远程解锁 PX4 飞控节点

用法：
    ros2 run rover_bringup arm_rover
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleCommand


class ArmRoverNode(Node):
    def __init__(self) -> None:
        super().__init__('arm_rover')
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_px4
        )
        self.get_logger().info('arm_rover：发布解锁指令...')
        self._send(400, 1.0, 0.0)  # 400 = MAV_CMD_COMPONENT_ARM_DISARM, param1=1
        self.get_logger().info('arm_rover：已发送 ARM 指令，3s 后退出。')

    def _send(self, command: int, p1: float = 0.0, p2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.from_external = True
        # 多次发送保证到达
        for _ in range(5):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmRoverNode()
    rclpy.spin_once(node, timeout_sec=3.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)