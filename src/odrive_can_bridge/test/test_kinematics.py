import math

from odrive_can_bridge.kinematics import (
    body_from_wheels,
    diagonal_covariance,
    integrate_pose,
    twist_to_wheel_rads,
    vehicle_wheel_speeds,
)


def test_straight_and_spin_commands():
    radius = 0.085
    track = 0.40
    directions = [1.0, 1.0, 1.0, 1.0]
    straight = twist_to_wheel_rads(1.0, 0.0, radius, track, directions, 1.2, 1.2, 20.0)
    assert straight[0] == straight[1]
    assert abs(straight[0] - straight[2]) < 1e-9
    assert abs(straight[0] - 1.0 / radius) < 1e-9

    spin = twist_to_wheel_rads(0.0, 1.0, radius, track, directions, 1.2, 1.2, 20.0)
    assert spin[0] < 0.0
    assert spin[2] > 0.0
    assert abs(spin[0] + spin[2]) < 1e-9


def test_direction_flips_only_that_wheel():
    base = twist_to_wheel_rads(0.5, 0.0, 0.1, 0.4, [1, 1, 1, 1], 1.2, 1.2, 20.0)
    flipped = twist_to_wheel_rads(0.5, 0.0, 0.1, 0.4, [1, 1, -1, -1], 1.2, 1.2, 20.0)
    assert flipped[0] == base[0]
    assert flipped[2] == -base[2]


def test_yaw_limit_is_body_rate_not_wheel_rate():
    wheels = twist_to_wheel_rads(0.0, 68.0, 0.085, 0.40, [1, 1, 1, 1], 1.2, 1.2, 20.0)
    assert max(abs(v) for v in wheels) <= 20.0 + 1e-9
    left, right = vehicle_wheel_speeds(wheels, [1, 1, 1, 1])
    _, wz = body_from_wheels(left, right, 0.085, 0.40)
    assert abs(wz) <= 1.2 + 1e-6


def test_feedback_sign_and_pose_integration():
    left, right = vehicle_wheel_speeds([1.0, 1.0, -1.0, -1.0], [1, 1, -1, -1])
    assert abs(left - 1.0) < 1e-9
    assert abs(right - 1.0) < 1e-9
    vx, wz = body_from_wheels(left, right, 0.1, 0.4)
    assert abs(vx - 0.1) < 1e-9
    assert abs(wz) < 1e-9
    x, y, yaw = integrate_pose(0.0, 0.0, 0.0, 1.0, 0.0, 1.0)
    assert abs(x - 1.0) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9
    _, _, yaw = integrate_pose(0.0, 0.0, 0.0, 0.0, math.pi / 2.0, 1.0)
    assert abs(yaw - math.pi / 2.0) < 1e-9


def test_covariance_length():
    cov = diagonal_covariance([0.05, 0.05, 1e6, 1e6, 1e6, 0.1])
    assert len(cov) == 36
    assert cov[0] == 0.05
    assert cov[35] == 0.1


def test_only_velocity_node_publishes_wheel_cmd():
    from pathlib import Path
    src = Path(__file__).resolve().parents[2]
    publishers = []
    for path in src.rglob('*.py'):
        if 'test' in path.parts:
            continue
        text = path.read_text(encoding='utf-8')
        if "create_publisher(Float32MultiArray, '/wheel_cmd'" in text:
            publishers.append(path.name)
    assert publishers == ['odrive_velocity_control_node.py']
