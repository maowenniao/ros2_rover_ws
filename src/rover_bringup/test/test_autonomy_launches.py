from pathlib import Path


def test_mapping_and_navigation_do_not_both_publish_map_odom():
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    mapping = (launch_dir / 'mapping.launch.py').read_text(encoding='utf-8')
    navigation = (launch_dir / 'navigation.launch.py').read_text(encoding='utf-8')
    assert 'mapping.launch.py' in mapping
    assert 'navigation.launch.py' not in mapping
    assert 'navigation.launch.py' in navigation
    assert 'mapping.launch.py' not in navigation
    assert "publish_odom_tf': 'false'" in mapping
    assert "publish_odom_tf': 'false'" in navigation
    assert 'enable_safety' in mapping
    assert 'enable_safety' in navigation


def test_launch_descriptions_build():
    import importlib.util
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    for name in ('mapping.launch.py', 'navigation.launch.py', 'full_system.launch.py'):
        path = launch_dir / name
        spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        assert module.generate_launch_description() is not None


def test_depth_scan_uses_optical_frame():
    text = Path(__file__).resolve().parents[2].joinpath(
        'rover_mapping', 'launch', 'depth_to_scan.launch.py').read_text(encoding='utf-8')
    assert 'd435i_depth_optical_frame' in text
    assert '/d435i/d435i/depth/image_rect_raw' in text
    assert 'camera_link' not in text
