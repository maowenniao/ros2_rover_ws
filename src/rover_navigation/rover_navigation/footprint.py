"""由 rover_geometry.yaml 生成 Nav2 矩形足迹和速度上限。"""
from __future__ import annotations

import re
from typing import Dict


def limit_geometry(geometry: Dict, max_linear: float, max_angular: float) -> Dict:
    """户外速度上限。不会超过底盘几何文件里的值。"""
    limited = dict(geometry)
    limited['max_linear_vel'] = min(abs(float(max_linear)), abs(float(geometry['max_linear_vel'])))
    limited['max_angular_vel'] = min(abs(float(max_angular)), abs(float(geometry['max_angular_vel'])))
    return limited


def footprint_polygon(geometry: Dict, margin: float = 0.05) -> str:
    half_length = float(geometry['wheel_base']) / 2.0 + margin
    half_width = float(geometry['track_width']) / 2.0 + margin
    corners = [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ]
    return '[' + ', '.join(f'[{x:.3f}, {y:.3f}]' for x, y in corners) + ']'


def velocity_tokens(geometry: Dict) -> Dict[str, str]:
    linear = float(geometry['max_linear_vel'])
    angular = float(geometry['max_angular_vel'])
    max_vx = min(0.8, linear)
    max_wz = min(1.0, angular)
    return {
        'FOOTPRINT': footprint_polygon(geometry),
        'DESIRED_VX': f'{min(0.45, linear):.2f}',
        'MAX_VX': f'{max_vx:.2f}',
        'MIN_VX': f'{-min(0.4, linear):.2f}',
        'MAX_WZ': f'{max_wz:.2f}',
    }


def materialize_nav_params(template: str, geometry: Dict) -> str:
    text = template
    for key, value in velocity_tokens(geometry).items():
        text = text.replace(f'__{key}__', value)
    leftover = re.findall(r'__[A-Z0-9_]+__', text)
    if leftover:
        raise ValueError(f'导航参数模板还有未替换的占位符: {leftover}')
    return text
