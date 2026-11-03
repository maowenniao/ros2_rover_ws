from pathlib import Path

import yaml


def test_ekf_uses_wheel_odom_and_px4_imu():
    path = Path(__file__).resolve().parents[1] / 'config' / 'ekf.yaml'
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    params = data['ekf_filter_node']['ros__parameters']
    assert params['publish_tf'] is True
    assert params['frequency'] <= 20.0
    assert params['odom0'] == '/wheel_odom'
    assert params['imu0'] == '/px4/imu'
    assert 'd435i' not in params['imu0']
    assert params['odom0_config'][0] is True
    assert params['odom0_config'][5] is True
    # 第 12 项是 vyaw，IMU 用它，不用绝对航向。
    assert params['imu0_config'][11] is True
    assert params['imu0_config'][5] is False


def test_outdoor_map_ekf_fuses_gps_and_px4_heading():
    root = Path(__file__).resolve().parents[1]
    local = yaml.safe_load((root / 'config' / 'ekf.yaml').read_text(encoding='utf-8'))
    local_imu = local['ekf_filter_node']['ros__parameters']['imu0_config']
    assert local_imu[5] is False
    assert local_imu[11] is True

    data = yaml.safe_load((root / 'config' / 'ekf_map.yaml').read_text(encoding='utf-8'))
    params = data['ekf_filter_node_map']['ros__parameters']
    assert params['world_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_link_frame'] == 'base_link'
    assert params['publish_tf'] is True
    assert params['two_d_mode'] is True
    assert params['frequency'] <= 20.0
    assert params['odom0'] == '/wheel_odom'
    assert params['odom0_config'][0] is False
    assert params['odom0_config'][6] is True
    assert params['odom1'] == '/odometry/gps'
    assert params['odom1_config'][0] is True
    assert params['odom1_config'][1] is True
    assert params['odom1_config'][5] is False
    assert params['imu0'] == '/px4/imu'
    assert params['imu0_config'][5] is True
    assert params['imu0_config'][11] is True

    navsat = yaml.safe_load((root / 'config' / 'navsat.yaml').read_text(encoding='utf-8'))
    gps = navsat['navsat_transform']['ros__parameters']
    assert gps['use_odometry_yaw'] is False
    assert gps['yaw_offset'] == 0.0
    assert gps['magnetic_declination_radians'] == 0.0
    assert gps['zero_altitude'] is True
    assert gps['wait_for_datum'] is False
    assert gps['broadcast_cartesian_transform'] is False
    assert gps['frequency'] <= 20.0
