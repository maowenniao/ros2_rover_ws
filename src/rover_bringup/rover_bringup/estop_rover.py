#!/usr/bin/env python3
"""急停：/estop、电机去使能、/cmd_vel 置 0、PX4 disarm。

`estop_rover` 发一次。`estop_action_node` 常驻，之后任何 /estop=true 都走同一动作。
ODrive 总线上的 Estop 帧由 odrive_can_node 在收到 /estop 时发送。
"""
import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand
from rclpy.node import Node
from std_msgs.msg import Bool

from rover_bringup.px4_commands import bool_msg, latched_qos, publish_times, px4_qos, vehicle_command


def stop_and_disarm(node: Node, cmd_pub, enable_pub, px4_pub) -> None:
    publish_times(node, cmd_pub, Twist())
    publish_times(node, enable_pub, bool_msg(False))
    # 400 = MAV_CMD_COMPONENT_ARM_DISARM，param2=21196 为强制上锁。
    publish_times(node, px4_pub, vehicle_command(node, 400, 0.0, 21196.0))


class EstopActionNode(Node):
    def __init__(self) -> None:
        super().__init__('estop_action')
        self.px4_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos())
        self.enable_pub = self.create_publisher(Bool, '/odrive_enable', latched_qos())
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/estop', self._on_estop, latched_qos())
        self._active = False
        self.get_logger().info('急停动作节点已启动')

    def _on_estop(self, msg: Bool) -> None:
        if not msg.data:
            self._active = False
            return
        if self._active:
            return
        self._active = True
        self.get_logger().error('执行急停：速度清零并 disarm PX4')
        # 回调里不要再 spin。多发几次，执行器下一轮会送出去。
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            self.enable_pub.publish(bool_msg(False))
            self.px4_pub.publish(vehicle_command(self, 400, 0.0, 21196.0))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('estop_rover')
    px4_pub = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos())
    estop_pub = node.create_publisher(Bool, '/estop', latched_qos())
    enable_pub = node.create_publisher(Bool, '/odrive_enable', latched_qos())
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    node.get_logger().error('发送急停')
    publish_times(node, estop_pub, bool_msg(True))
    stop_and_disarm(node, cmd_pub, enable_pub, px4_pub)
    node.destroy_node()
    rclpy.shutdown()


def action_main(args=None) -> None:
    rclpy.init(args=args)
    node = EstopActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
