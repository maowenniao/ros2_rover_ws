#!/usr/bin/env python3
"""解锁 PX4，并允许 ODrive 进入闭环。

电机节点必须已经在运行。本命令先清掉急停锁存，再发 /odrive_enable。
"""
import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand
from rclpy.node import Node
from std_msgs.msg import Bool

from rover_bringup.px4_commands import bool_msg, latched_qos, publish_times, px4_qos, vehicle_command


class ArmRoverNode(Node):
    def __init__(self) -> None:
        super().__init__('arm_rover')
        px4_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos())
        estop_pub = self.create_publisher(Bool, '/estop', latched_qos())
        enable_pub = self.create_publisher(Bool, '/odrive_enable', latched_qos())
        self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('解除急停锁存，使能 ODrive，并解锁 PX4')
        publish_times(self, estop_pub, bool_msg(False))
        publish_times(self, enable_pub, bool_msg(True))
        publish_times(self, px4_pub, vehicle_command(self, 400, 1.0, 0.0))
        self.get_logger().info('ARM 已发送。轮子离地确认方向之前不要给速度。')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmRoverNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
