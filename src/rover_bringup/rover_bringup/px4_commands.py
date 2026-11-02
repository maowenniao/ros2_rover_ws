"""PX4 解锁 / 上锁，以及给底盘用的锁存话题。"""
from __future__ import annotations

from px4_msgs.msg import VehicleCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def latched_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def vehicle_command(node: Node, command: int, param1: float, param2: float = 0.0) -> VehicleCommand:
    msg = VehicleCommand()
    msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    msg.command = command
    msg.param1 = param1
    msg.param2 = param2
    msg.target_system = 1
    msg.target_component = 1
    msg.source_system = 255
    msg.source_component = 0
    msg.from_external = True
    return msg


def publish_times(node: Node, publisher, message, count: int = 5, pause: float = 0.1) -> None:
    import rclpy
    for _ in range(count):
        if hasattr(message, 'timestamp'):
            message.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=pause)


def bool_msg(value: bool) -> Bool:
    msg = Bool()
    msg.data = value
    return msg
