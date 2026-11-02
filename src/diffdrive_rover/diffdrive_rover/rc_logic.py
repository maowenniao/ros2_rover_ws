"""遥控器摇杆、模式切换和急停判断。不依赖 rclpy，方便单测。"""
from __future__ import annotations

import math
from typing import Optional, Tuple


def apply_deadband(value: float, deadband: float) -> float:
    """死区外重新标度到 [-1, 1]，死区内为 0。"""
    if not math.isfinite(value):
        return 0.0
    width = abs(deadband)
    if width >= 1.0:
        return 0.0
    if abs(value) <= width:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - width) / (1.0 - width)


def apply_expo(value: float, expo: float) -> float:
    """expo=0 为线性，expo=1 为三次曲线。"""
    if not math.isfinite(value):
        return 0.0
    gain = min(1.0, max(0.0, expo))
    return (1.0 - gain) * value + gain * value * value * value


def sticks_to_body_velocity(
    pitch: float,
    yaw: float,
    deadband: float,
    expo: float,
    max_linear_vel: float,
    max_angular_vel: float,
) -> Tuple[float, float]:
    """PX4 摇杆转成 ROS 机体速度。

    pitch 正方向是前进。yaw 正方向是从上往下看顺时针，ROS 的角速度正方向相反。
    """
    forward = apply_expo(apply_deadband(pitch, deadband), expo)
    clockwise = apply_expo(apply_deadband(yaw, deadband), expo)
    vx = forward * abs(max_linear_vel)
    wz = -clockwise * abs(max_angular_vel)
    return vx, wz


def update_mode(current: str, switch_value: Optional[float], threshold: float) -> str:
    """三挡开关：高于阈值手动，低于负阈值自动，中间或无效则保持。"""
    if switch_value is None or not math.isfinite(switch_value):
        return current
    if switch_value > threshold:
        return 'manual'
    if switch_value < -threshold:
        return 'auto'
    return current


def select_velocity(
    mode: str,
    manual_vx: float,
    manual_wz: float,
    auto_vx: float,
    auto_wz: float,
    auto_age: Optional[float],
    auto_timeout: float,
) -> Tuple[float, float, str]:
    """手动用摇杆。自动转发 Nav2，超时则发 0，避免锁存最后一条导航速度。"""
    if mode == 'manual':
        return manual_vx, manual_wz, 'manual'
    if auto_age is None or auto_age > auto_timeout:
        return 0.0, 0.0, 'auto_timeout'
    return auto_vx, auto_wz, 'auto'


def rc_link_lost(
    valid: bool,
    age_sec: Optional[float],
    timeout: float,
    signal_lost: bool,
    ever_seen: bool = True,
) -> bool:
    """从未收到过遥控时不判丢失，避免飞控还没连上就把导航急停。"""
    if signal_lost:
        return True
    if not ever_seen:
        return False
    if age_sec is None or age_sec > timeout:
        return True
    return not valid


def estop_requested(
    link_lost: bool,
    kill_switch: bool,
    estop_switch: Optional[float],
    estop_threshold: float,
    px4_failsafe: bool,
    geofence_breached: bool,
    battery_critical: bool,
) -> Optional[str]:
    """返回急停原因；没有原因则返回 None。"""
    if link_lost:
        return 'rc_lost'
    if kill_switch:
        return 'kill_switch'
    if estop_switch is not None and math.isfinite(estop_switch) and estop_switch > estop_threshold:
        return 'estop_switch'
    if geofence_breached:
        return 'geofence'
    if battery_critical:
        return 'battery'
    if px4_failsafe:
        return 'px4_failsafe'
    return None


def frd_angular_to_flu(xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """PX4 机体系 FRD 角速度转到 ROS 机体系 FLU。航向角速度变号。"""
    return (float(xyz[0]), -float(xyz[1]), -float(xyz[2]))
