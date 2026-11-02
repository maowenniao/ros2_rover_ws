"""四轮差速运动学。左右各两轮同速，方向系数由几何参数给出。"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def twist_to_wheel_rads(
    vx: float,
    wz: float,
    wheel_radius: float,
    track_width: float,
    directions: Sequence[float],
    max_linear_vel: float,
    max_angular_vel: float,
    max_wheel_speed: float,
) -> List[float]:
    """机体速度转为四个电机角速度 (rad/s)，顺序 [左前, 左后, 右前, 右后]。

    directions 与轮序一致。电机装反的那一侧填 -1，上车前必须实测。
    """
    if wheel_radius <= 0.0 or track_width <= 0.0:
        raise ValueError('wheel_radius 和 track_width 必须为正')
    if len(directions) != 4:
        raise ValueError('wheel_directions 必须是 4 个数')
    vx = clamp(float(vx), -abs(max_linear_vel), abs(max_linear_vel))
    wz = clamp(float(wz), -abs(max_angular_vel), abs(max_angular_vel))
    v_left = vx - wz * track_width / 2.0
    v_right = vx + wz * track_width / 2.0
    wheel_limit = abs(max_wheel_speed)
    w_left = clamp(v_left / wheel_radius, -wheel_limit, wheel_limit)
    w_right = clamp(v_right / wheel_radius, -wheel_limit, wheel_limit)
    unsigned = [w_left, w_left, w_right, w_right]
    return [unsigned[i] * float(directions[i]) for i in range(4)]


def vehicle_wheel_speeds(motor_rad_s: Sequence[float], directions: Sequence[float]) -> Tuple[float, float]:
    """把电机反馈乘方向系数，得到车体意义下的左右轮平均角速度 (rad/s)。"""
    if len(motor_rad_s) < 4 or len(directions) != 4:
        raise ValueError('需要 4 个轮速和 4 个方向系数')
    signed = [float(motor_rad_s[i]) * float(directions[i]) for i in range(4)]
    left = (signed[0] + signed[1]) / 2.0
    right = (signed[2] + signed[3]) / 2.0
    return left, right


def body_from_wheels(w_left: float, w_right: float, wheel_radius: float, track_width: float) -> Tuple[float, float]:
    """左右轮角速度 (rad/s) 转为机体 vx (m/s) 与 wz (rad/s)。"""
    if wheel_radius <= 0.0 or track_width <= 0.0:
        raise ValueError('wheel_radius 和 track_width 必须为正')
    v_left = w_left * wheel_radius
    v_right = w_right * wheel_radius
    vx = (v_left + v_right) / 2.0
    wz = (v_right - v_left) / track_width
    return vx, wz


def integrate_pose(x: float, y: float, theta: float, vx: float, wz: float, dt: float) -> Tuple[float, float, float]:
    """在机体速度上做一小段圆弧积分，返回新的 x, y, yaw。"""
    if dt <= 0.0:
        return x, y, theta
    if abs(wz) < 1e-6:
        x += vx * math.cos(theta) * dt
        y += vx * math.sin(theta) * dt
    else:
        x += (vx / wz) * (math.sin(theta + wz * dt) - math.sin(theta))
        y += (vx / wz) * (-math.cos(theta + wz * dt) + math.cos(theta))
    yaw = theta + wz * dt
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return x, y, yaw


def diagonal_covariance(variances: Sequence[float]) -> List[float]:
    """6 个对角方差展开成 36 元协方差。"""
    if len(variances) != 6:
        raise ValueError('协方差对角需要 6 个数')
    cov = [0.0] * 36
    for index, value in enumerate(variances):
        cov[index * 6 + index] = float(value)
    return cov
