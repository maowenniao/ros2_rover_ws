#!/usr/bin/env python3
"""把航点一次性交给 Nav2 FollowWaypoints。循环只在任务成功后重发。"""
from __future__ import annotations

import math
from typing import List

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def yaw_to_quat(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def filter_close_waypoints(poses: List[PoseStamped], tolerance: float) -> List[PoseStamped]:
    """丢掉与上一个航点水平距离小于 tolerance 的点。tolerance<=0 时不筛选。"""
    if tolerance <= 0.0 or not poses:
        return list(poses)
    kept = [poses[0]]
    for pose in poses[1:]:
        previous = kept[-1].pose.position
        dx = pose.pose.position.x - previous.x
        dy = pose.pose.position.y - previous.y
        if math.hypot(dx, dy) >= tolerance:
            kept.append(pose)
    return kept


class WaypointMissionNode(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_mission_node')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('arrival_tolerance', 0.30)
        self.declare_parameter('loop', True)
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.mission: List[PoseStamped] = []
        self._goal_handle = None
        self._sending = False
        self._finished = False
        self._load_waypoints()
        self._timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'航点任务就绪：{len(self.mission)} 个目标，loop={self._loop()}'
        )

    def _loop(self) -> bool:
        return as_bool(self.get_parameter('loop').value)

    def _load_waypoints(self) -> None:
        path = self.get_parameter('waypoints_file').value
        tolerance = float(self.get_parameter('arrival_tolerance').value)
        poses: List[PoseStamped] = []
        if not path:
            self.get_logger().warn('未配置 waypoints_file，使用地图原点')
            poses = [self._make_pose(0.0, 0.0, 0.0)]
        else:
            with open(path, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            for waypoint in data.get('waypoints', []):
                poses.append(self._make_pose(
                    float(waypoint['x']), float(waypoint['y']), float(waypoint.get('yaw', 0.0))))
        self.mission = filter_close_waypoints(poses, tolerance)

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quat(yaw)
        return pose

    def _tick(self) -> None:
        if self._finished or self._sending or self._goal_handle is not None or not self.mission:
            return
        if not self.client.wait_for_server(timeout_sec=0.2):
            return
        self._sending = True
        goal = FollowWaypoints.Goal()
        now = self.get_clock().now().to_msg()
        for pose in self.mission:
            pose.header.stamp = now
        # 0 表示走完一遍。循环由本节点在成功后再发，避免定时器叠成多份目标。
        goal.number_of_loops = 0
        goal.poses = self.mission
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        self._sending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('航点任务被拒绝，稍后再试')
            self._goal_handle = None
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future) -> None:
        self._goal_handle = None
        self.get_logger().info('一轮航点结束')
        if not self._loop():
            self._finished = True
            self._timer.cancel()
            self.get_logger().info('非循环模式，任务停止')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
