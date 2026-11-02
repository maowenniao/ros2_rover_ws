"""把导航/遥控速度限制成底盘可以执行的 Twist。"""
from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def front_min_range(scan: LaserScan, sector_rad: float) -> float:
    """前方扇区内的最近有效距离。没有有效点时返回 +inf。"""
    values = []
    angle = scan.angle_min
    for distance in scan.ranges:
        if abs(angle) <= sector_rad and math.isfinite(distance):
            if scan.range_min <= distance <= scan.range_max:
                values.append(distance)
        angle += scan.angle_increment
    return min(values, default=float('inf'))


def limit_twist(
    cmd: Twist,
    cmd_age: Optional[float],
    cmd_timeout: float,
    scan_age: Optional[float],
    scan_timeout: float,
    require_scan: bool,
    min_front: float,
    stop_distance: float,
    slow_distance: float,
) -> Twist:
    """指令超时、扫描超时或前方过近时输出 0。倒车不受前方障碍缩放。"""
    out = Twist()
    if cmd_age is None or cmd_age > cmd_timeout:
        return out
    if require_scan and (scan_age is None or scan_age > scan_timeout):
        return out
    if min_front <= stop_distance:
        factor = 0.0
    elif min_front < slow_distance:
        span = slow_distance - stop_distance
        factor = 0.0 if span <= 0.0 else max(0.0, (min_front - stop_distance) / span)
    else:
        factor = 1.0
    if cmd.linear.x > 0.0:
        out.linear.x = cmd.linear.x * factor
        out.angular.z = cmd.angular.z * (0.25 + 0.75 * factor)
    else:
        out.linear.x = cmd.linear.x
        out.angular.z = cmd.angular.z
    out.linear.y = cmd.linear.y * factor if cmd.linear.x > 0.0 else cmd.linear.y
    return out
