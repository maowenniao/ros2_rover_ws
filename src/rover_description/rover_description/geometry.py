"""读取 rover_geometry.yaml。launch 与测试共用。"""
from __future__ import annotations

import os
from typing import Dict, Optional

import yaml

REQUIRED_KEYS = (
    'wheel_radius',
    'track_width',
    'wheel_base',
    'wheel_directions',
    'max_wheel_speed',
    'max_linear_vel',
    'max_angular_vel',
    'gps_x',
    'gps_y',
    'gps_z',
)


def default_geometry_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('rover_description'), 'config', 'rover_geometry.yaml')


def declare_geometry(node) -> None:
    """声明几何参数。节点再加载 rover_geometry.yaml 时，未声明的键不会把启动打崩。"""
    node.declare_parameter('wheel_radius', 0.085)
    node.declare_parameter('track_width', 0.40)
    node.declare_parameter('wheel_base', 0.50)
    node.declare_parameter('wheel_directions', [1.0, 1.0, 1.0, 1.0])
    node.declare_parameter('max_wheel_speed', 20.0)
    node.declare_parameter('max_linear_vel', 1.2)
    node.declare_parameter('max_angular_vel', 1.2)
    node.declare_parameter('gps_x', 0.0)
    node.declare_parameter('gps_y', 0.0)
    node.declare_parameter('gps_z', 0.15)


def load_geometry(path: Optional[str] = None) -> Dict:
    geometry_path = path or default_geometry_path()
    with open(geometry_path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    try:
        params = data['/**']['ros__parameters']
    except (TypeError, KeyError) as exc:
        raise ValueError(f'{geometry_path} 不是 /**: ros__parameters 格式') from exc
    missing = [key for key in REQUIRED_KEYS if key not in params]
    if missing:
        raise ValueError(f'{geometry_path} 缺少 {missing}')
    if len(params['wheel_directions']) != 4:
        raise ValueError('wheel_directions 必须是 4 个数')
    return params
