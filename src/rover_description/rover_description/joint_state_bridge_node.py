#!/usr/bin/env python3
"""
joint_state_bridge_node
=======================

订阅 /wheel_state（sensor_msgs/JointState，来自 ODrive CAN 节点），
将 ODrive 节点 ID 重新映射到 URDF 中的关节名，发布 /joint_states。

关节映射：
    wheel_0 → lf_wheel_joint
    wheel_1 → lr_wheel_joint
    wheel_2 → rf_wheel_joint
    wheel_3 → rr_wheel_joint
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState


class JointStateBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_bridge_node')

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            JointState, '/wheel_state', self._cb, qos_reliable,
        )
        self.pub = self.create_publisher(
            JointState, '/joint_states', qos_reliable,
        )

        self._map = {
            'wheel_0': 'lf_wheel_joint',
            'wheel_1': 'lr_wheel_joint',
            'wheel_2': 'rf_wheel_joint',
            'wheel_3': 'rr_wheel_joint',
        }
        self._position = {n: 0.0 for n in self._map.values()}
        self.get_logger().info('joint_state_bridge_node 已启动')

    def _cb(self, msg: JointState) -> None:
        out = JointState()
        out.header = msg.header

        for name in msg.name:
            target = self._map.get(name)
            if target is None:
                continue
            try:
                idx = msg.name.index(name)
            except ValueError:
                continue
            new_pos = (
                float(msg.position[idx]) if idx < len(msg.position) else 0.0
            )
            self._position[target] = new_pos

        out.name = list(self._position.keys())
        out.position = list(self._position.values())
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()