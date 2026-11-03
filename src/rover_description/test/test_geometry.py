import shutil
import subprocess
from pathlib import Path

import pytest

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
    for key in ('gps_x', 'gps_y', 'gps_z'):
        assert isinstance(params[key], (int, float))


def test_urdf_has_configurable_gps_link():
    urdf = Path(__file__).resolve().parents[1].joinpath('urdf', 'rover.urdf.xacro').read_text(encoding='utf-8')
    assert 'name="gps_link"' in urdf
    assert '$(arg gps_x)' in urdf
    assert '$(arg gps_y)' in urdf
    assert '$(arg gps_z)' in urdf


def test_xacro_places_gps_link_from_arguments():
    xacro = shutil.which('xacro')
    if xacro is None:
        pytest.skip('未安装 xacro')
    urdf = Path(__file__).resolve().parents[1].joinpath('urdf', 'rover.urdf.xacro')
    rendered = subprocess.check_output(
        [xacro, str(urdf), 'gps_x:=0.2', 'gps_y:=-0.1', 'gps_z:=0.3'],
        text=True,
    )
    assert '<link name="gps_link">' in rendered
    assert 'xyz="0.2 -0.1 0.3"' in rendered
