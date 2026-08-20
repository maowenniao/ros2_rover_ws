#!/usr/bin/env python3
"""
estop_rover - 紧急停止：发送 PX4 disarm + ODrive Estop 指令
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleCommand
from std_msgs.msg import Float32MultiArray


class EstopRoverNode(Node):
    def __init__(self) -> None:
        super().__init__('estop_rover')

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
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_px4
        )
        self.wheel_pub = self.create_publisher(
            Float32MultiArray, '/wheel_cmd', qos_reliable
        )

        # 1) PX4 Disarm
        now = self.get_clock().now()
        disarm = VehicleCommand()
        disarm.timestamp = int(now.nanoseconds / 1000)
        disarm.command = 400  # COMPONENT_ARM_DISARM
        disarm.param1 = 0.0   # disarm
        disarm.param2 = 21196.0  # MAV_CMD_COMPONENT_ARM_DISARM_FORCE
        disarm.target_system = 1
        disarm.target_component = 1
        disarm.source_system = 255
        disarm.source_component = 0
        disarm.from_external = True
        for _ in range(5):
            self.cmd_pub.publish(disarm)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 2) ODrive wheel_cmd = 0（让 ODrive CAN 节点发 0 速度）
        zero = Float32MultiArray()
        zero.data = [0.0, 0.0, 0.0, 0.0]
        for _ in range(10):
            self.wheel_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn('estop_rover：已发送 DISARM + 0 速度指令')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstopRoverNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)