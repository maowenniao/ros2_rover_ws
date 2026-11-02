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
