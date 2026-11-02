from pathlib import Path


def test_camera_launch_connects_optical_frames():
    text = (Path(__file__).resolve().parents[1] / 'launch' / 'd435i.launch.py').read_text(encoding='utf-8')
    for frame in (
        'd435i_link',
        'd435i_depth_optical_frame',
        'd435i_color_optical_frame',
        'd435i_imu_optical_frame',
    ):
        assert frame in text
    assert 'publish_tf' in text
    assert '640x480x15' in text
    config = Path(__file__).resolve().parents[1] / 'config' / 'd435i.yaml'
    body = config.read_text(encoding='utf-8')
    assert 'align_depth.enable: true' in body
    assert 'unite_imu_method: 0' in body
    assert 'enable_gyro: false' in body
