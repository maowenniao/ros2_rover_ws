import math

from diffdrive_rover.rc_logic import (
    apply_deadband,
    apply_expo,
    estop_requested,
    frd_angular_to_flu,
    rc_link_lost,
    select_velocity,
    sticks_to_body_velocity,
    update_mode,
)


def test_deadband_and_expo():
    assert apply_deadband(0.02, 0.06) == 0.0
    assert abs(apply_deadband(1.0, 0.06) - 1.0) < 1e-9
    assert apply_expo(0.5, 0.0) == 0.5
    softened = apply_expo(0.5, 1.0)
    assert abs(softened - 0.125) < 1e-9


def test_stick_signs_and_limits():
    vx, wz = sticks_to_body_velocity(1.0, 0.0, 0.0, 0.0, 1.2, 1.0)
    assert abs(vx - 1.2) < 1e-9
    assert wz == 0.0
    _, wz = sticks_to_body_velocity(0.0, 1.0, 0.0, 0.0, 1.2, 1.0)
    assert abs(wz + 1.0) < 1e-9
    vx, _ = sticks_to_body_velocity(0.03, 0.0, 0.06, 0.0, 1.2, 1.0)
    assert vx == 0.0


def test_mode_switch_holds_center():
    assert update_mode('auto', 0.8, 0.3) == 'manual'
    assert update_mode('manual', -0.8, 0.3) == 'auto'
    assert update_mode('manual', 0.0, 0.3) == 'manual'
    assert update_mode('auto', math.nan, 0.3) == 'auto'


def test_auto_timeout_does_not_repeat_nav_command():
    assert select_velocity('manual', 0.2, -0.1, 0.9, 0.9, 0.0, 0.5)[:2] == (0.2, -0.1)
    assert select_velocity('auto', 0.2, 0.0, 0.4, 0.1, 0.1, 0.5)[:2] == (0.4, 0.1)
    assert select_velocity('auto', 0.2, 0.0, 0.4, 0.1, None, 0.5)[:2] == (0.0, 0.0)
    assert select_velocity('auto', 0.2, 0.0, 0.4, 0.1, 1.0, 0.5)[:2] == (0.0, 0.0)


def test_estop_reasons():
    assert rc_link_lost(True, None, 0.5, False, ever_seen=False) is False
    assert rc_link_lost(True, 0.1, 0.5, False) is False
    assert rc_link_lost(False, 0.1, 0.5, False) is True
    assert rc_link_lost(True, 1.0, 0.5, False) is True
    assert estop_requested(True, False, None, 0.5, False, False, False) == 'rc_lost'
    assert estop_requested(False, True, None, 0.5, False, False, False) == 'kill_switch'
    assert estop_requested(False, False, 0.8, 0.5, False, False, False) == 'estop_switch'
    assert estop_requested(False, False, None, 0.5, False, True, False) == 'geofence'
    assert estop_requested(False, False, math.nan, 0.5, False, False, False) is None


def test_yaw_rate_sign_matches_ros():
    # FRD 正航向角速度是从上往下顺时针，FLU 应为负。
    assert frd_angular_to_flu((0.0, 0.2, 0.4)) == (0.0, -0.2, -0.4)
