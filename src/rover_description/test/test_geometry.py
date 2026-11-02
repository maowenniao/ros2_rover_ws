from pathlib import Path

from rover_description.geometry import REQUIRED_KEYS, load_geometry


def test_geometry_yaml_is_one_source_and_yaw_limit_is_sane():
    path = Path(__file__).resolve().parents[1] / 'config' / 'rover_geometry.yaml'
    params = load_geometry(str(path))
    assert set(REQUIRED_KEYS) <= set(params)
    assert len(params['wheel_directions']) == 4
    assert params['wheel_radius'] > 0.0
    assert params['track_width'] > 0.0
    # 68 rad/s 是把轮子角速度误当成车体 yaw。车体上限必须是个能开的数。
    assert params['max_angular_vel'] < 5.0
    assert params['max_linear_vel'] < params['max_wheel_speed'] * params['wheel_radius']
