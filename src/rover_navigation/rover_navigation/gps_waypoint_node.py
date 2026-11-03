#!/usr/bin/env python3
"""把经纬度航点交给 Nav2 的 follow_gps_waypoints。

Nav2 内部调用 navsat 的 /fromLL，把经纬度变成地图坐标。
默认不自动出发。确认 /gps/fix 之后再发 /gps_waypoints/start，或启动时 follow_gps:=true。
没有定位时只等待，不会发目标。
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import FollowGPSWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool

from rover_navigation.gps_waypoints import (
    GpsWaypoint,
    as_bool,
    load_gps_waypoints,
    should_send,
    yaw_to_quat,
)


class GpsWaypointNode(Node):
    def __init__(self) -> None:
        super().__init__('gps_waypoint_node')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('autostart', 'false')
        self.declare_parameter('start_delay_sec', 8.0)
        self.declare_parameter('max_attempts', 6)
        self._delay = float(self.get_parameter('start_delay_sec').value)
        self._max_attempts = int(self.get_parameter('max_attempts').value)
        self._mission: List[GpsWaypoint] = []
        self._requested = as_bool(self.get_parameter('autostart').value)
        self._sent = False
        self._sending = False
        self._finished = False
        self._attempts = 0
        self._gps_ok = False
        self._gps_since = None
        self._last_fix = None
        self._sent_at = None
        self._load()
        self._client = ActionClient(self, FollowGPSWaypoints, '/follow_gps_waypoints')
        self.create_subscription(NavSatFix, '/gps/fix', self._on_fix, 10)
        self.create_subscription(Bool, '/gps_waypoints/start', self._on_start, 10)
        self.create_timer(0.5, self._tick)
        if self._mission:
            self.get_logger().info(
                f'GPS 航点已加载 {len(self._mission)} 个。'
                '确认 /gps/fix 后发 /gps_waypoints/start，或启动时加 follow_gps:=true'
            )
        else:
            self.get_logger().info('没有 GPS 航点，不会出发')

    def _load(self) -> None:
        path = str(self.get_parameter('waypoints_file').value)
        if not path:
            return
        try:
            self._mission = load_gps_waypoints(path)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'读航点失败：{exc}')
            self._mission = []

    def _on_fix(self, msg: NavSatFix) -> None:
        if int(msg.status.status) <= NavSatStatus.STATUS_NO_FIX:
            self._gps_ok = False
            return
        now = self.get_clock().now()
        if self._gps_since is None:
            self._gps_since = now
        self._last_fix = now
        self._gps_ok = True

    def _on_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        self._requested = True
        self._sent = False
        self._sending = False
        self._finished = False
        self._attempts = 0
        self.get_logger().info('收到出发请求，等定位稳定后发给 Nav2')

    def _seconds_since_fix(self) -> Optional[float]:
        if self._gps_since is None:
            return None
        return (self.get_clock().now() - self._gps_since).nanoseconds / 1e9

    def _fix_is_fresh(self) -> bool:
        if not self._gps_ok or self._last_fix is None:
            return False
        age = (self.get_clock().now() - self._last_fix).nanoseconds / 1e9
        return age <= 2.0

    def _tick(self) -> None:
        if self._finished or self._sending or not self._mission:
            return
        if not should_send(
            self._requested,
            self._sent,
            self._fix_is_fresh(),
            self._seconds_since_fix(),
            self._delay,
        ):
            return
        if self._attempts >= self._max_attempts:
            self._finished = True
            self.get_logger().error('GPS 航点多次失败，停止。看看 /gps/fix 和 /odometry/gps')
            return
        if not self._client.wait_for_server(timeout_sec=0.1):
            return
        goal = FollowGPSWaypoints.Goal()
        goal.number_of_loops = 0
        goal.goal_index = 0
        goal.gps_poses = [self._to_geo(point) for point in self._mission]
        self._sending = True
        self._sent = True
        self._attempts += 1
        self._sent_at = self.get_clock().now()
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _to_geo(self, point: GpsWaypoint) -> GeoPose:
        pose = GeoPose()
        pose.position.latitude = point.latitude
        pose.position.longitude = point.longitude
        pose.position.altitude = point.altitude
        x, y, z, w = yaw_to_quat(point.yaw)
        pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        return pose

    def _goal_response(self, future) -> None:
        self._sending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('GPS 航点被拒绝，稍后重试')
            self._sent = False
            return
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future) -> None:
        wrapped = future.result()
        status = wrapped.status if wrapped is not None else GoalStatus.STATUS_ABORTED
        elapsed = 0.0
        if self._sent_at is not None:
            elapsed = (self.get_clock().now() - self._sent_at).nanoseconds / 1e9
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finished = True
            self.get_logger().info('GPS 航点走完')
            return
        # 很快失败通常是 navsat 还没把经纬度变到地图上。
        if elapsed < 3.0 and self._attempts < self._max_attempts:
            self.get_logger().warn('航点还没被接受，可能 navsat 还没对齐，稍后重试')
            self._sent = False
            return
        self._finished = True
        self.get_logger().error('GPS 航点失败，已停止')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
