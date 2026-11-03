"""经纬度航点。不依赖 rclpy。

yaw 用 ROS 航向：0 朝东，逆时针为正。省略 yaw 时朝向下一个点。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import yaml


def as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def yaw_to_quat(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def bearing_enu(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """从点 1 指向点 2 的 ENU 航向。正北是 +pi/2，正东是 0。"""
    north = math.radians(lat2 - lat1)
    east = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    return math.atan2(north, east)


@dataclass(frozen=True)
class GpsWaypoint:
    latitude: float
    longitude: float
    altitude: float
    yaw: float


def _finite_number(value, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} 不是数字') from exc
    if not math.isfinite(number):
        raise ValueError(f'{name} 不是有限数字')
    return number


def parse_gps_waypoints(data) -> List[GpsWaypoint]:
    if not isinstance(data, dict):
        raise ValueError('航点文件应为 waypoints 列表')
    raw = data.get('waypoints') or []
    if not isinstance(raw, list):
        raise ValueError('waypoints 必须是列表')
    points = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f'第 {index} 个航点不是字典')
        latitude = _finite_number(item.get('latitude'), 'latitude')
        longitude = _finite_number(item.get('longitude'), 'longitude')
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise ValueError(f'第 {index} 个航点的经纬度超出范围')
        altitude = 0.0
        if item.get('altitude') is not None:
            altitude = _finite_number(item.get('altitude'), 'altitude')
        yaw = None
        if item.get('yaw') is not None:
            yaw = _finite_number(item.get('yaw'), 'yaw')
        points.append((latitude, longitude, altitude, yaw))
    return assign_yaws(points)


def assign_yaws(points) -> List[GpsWaypoint]:
    count = len(points)
    assigned = []
    for index, point in enumerate(points):
        latitude, longitude, altitude, yaw = point
        if yaw is None:
            if count == 1:
                yaw = 0.0
            elif index < count - 1:
                nxt = points[index + 1]
                yaw = bearing_enu(latitude, longitude, nxt[0], nxt[1])
            else:
                prev = points[index - 1]
                yaw = bearing_enu(prev[0], prev[1], latitude, longitude)
        assigned.append(GpsWaypoint(latitude, longitude, altitude, yaw))
    return assigned


def load_gps_waypoints(path: str) -> List[GpsWaypoint]:
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    return parse_gps_waypoints(data)


def should_send(requested: bool, already_sent: bool, gps_fix_ok: bool,
                seconds_since_fix: Optional[float], delay_sec: float) -> bool:
    """有定位并且稳定一小段时间之后才发航点。没定位时不发。"""
    if not requested or already_sent or not gps_fix_ok:
        return False
    if seconds_since_fix is None:
        return False
    return seconds_since_fix >= delay_sec
