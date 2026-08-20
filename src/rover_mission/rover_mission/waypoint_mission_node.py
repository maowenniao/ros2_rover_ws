#!/usr/bin/env python3
"""航点任务节点：循环向 Nav2 follow_waypoints Action 发送目标序列。"""
import math
import yaml
from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WaypointMissionNode(Node):
    def __init__(self):
        super().__init__('waypoint_mission_node')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('arrival_tolerance', 0.30)
        self.declare_parameter('loop', True)
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.mission: List[PoseStamped] = []
        self.idx = 0
        self.goal_handle = None
        self.load_waypoints()
        if self.mission:
            self.create_timer(1.0, self._kickoff)
        self.get_logger().info(f'航点任务就绪：{len(self.mission)} 个目标，loop={self.get_parameter("loop").value}')

    def load_waypoints(self) -> None:
        path = self.get_parameter('waypoints_file').value
        if not path:
            self.get_logger().warn('未配置 waypoints_file，使用内置默认 0/0/0')
            self.mission = [self._make_pose(0.0, 0.0, 0.0)]
            return
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        for wp in data.get('waypoints', []):
            self.mission.append(self._make_pose(float(wp['x']), float(wp['y']), float(wp.get('yaw', 0.0))))

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quat(yaw)
        return pose

    def _kickoff(self) -> None:
        if self.goal_handle is not None:
            return
        if not self.client.wait_for_server(timeout_sec=1.0):
            return
        goal = FollowWaypoints.Goal()
        goal.poses = self.mission
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        self._kickoff_timer = self.create_timer(0.5, self._kickoff)

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('任务被拒绝，重试中...')
            self.goal_handle = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        self.get_logger().info('一轮任务完成')
        self.goal_handle = None
        if self.get_parameter('loop').value:
            return
        self.get_logger().info('非循环模式，任务结束')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
