import os
from pathlib import Path

import yaml

from rover_description.geometry import load_geometry
from rover_navigation.footprint import limit_geometry, materialize_nav_params, velocity_tokens


def test_materialized_limits_stay_inside_chassis():
    root = Path(__file__).resolve().parents[1]
    geometry = load_geometry(str(root.parents[0] / 'rover_description' / 'config' / 'rover_geometry.yaml'))
    template = (root / 'config' / 'nav2_params.yaml').read_text(encoding='utf-8')
    rendered = materialize_nav_params(template, geometry)
    data = yaml.safe_load(rendered)
    tokens = velocity_tokens(geometry)
    desired = float(tokens['DESIRED_VX'])
    max_vx = float(tokens['MAX_VX'])
    max_wz = float(tokens['MAX_WZ'])
    assert desired <= geometry['max_linear_vel']
    assert max_vx <= geometry['max_linear_vel']
    assert max_wz <= geometry['max_angular_vel']
    smoother = data['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] == max_vx
    assert abs(smoother['max_velocity'][2] - max_wz) < 1e-6
    assert smoother['min_velocity'][0] < 0.0
    footprint = data['local_costmap']['local_costmap']['ros__parameters']['footprint']
    assert footprint == tokens['FOOTPRINT']
    assert 'robot_radius' not in data['local_costmap']['local_costmap']['ros__parameters']
    bt = data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']
    assert os.path.isfile(bt)
    assert 'navigate_w_recovery.xml' not in bt
    cloud = data['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['cloud']['topic']
    assert cloud == '/d435i/d435i/depth/color/points'


def test_outdoor_nav_is_slow_and_does_not_use_amcl():
    root = Path(__file__).resolve().parents[1]
    geometry = load_geometry(str(root.parents[0] / 'rover_description' / 'config' / 'rover_geometry.yaml'))
    capped = limit_geometry(geometry, 0.30, 0.50)
    template = (root / 'config' / 'nav2_outdoor_params.yaml').read_text(encoding='utf-8')
    rendered = materialize_nav_params(template, capped)
    data = yaml.safe_load(rendered)
    assert 'amcl' not in data
    assert 'map_server' not in data
    global_costmap = data['global_costmap']['global_costmap']['ros__parameters']
    assert global_costmap['rolling_window'] is True
    assert 'static_layer' not in global_costmap['plugins']
    assert data['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is True
    tokens = velocity_tokens(capped)
    assert float(tokens['DESIRED_VX']) <= 0.30
    assert float(tokens['MAX_VX']) <= min(0.30, geometry['max_linear_vel'])
    smoother = data['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] <= geometry['max_linear_vel']
    assert smoother['max_velocity'][0] <= 0.30
    launch = (root / 'launch' / 'outdoor_navigation.launch.py').read_text(encoding='utf-8')
    assert 'map_server' not in launch
    assert 'amcl' not in launch
    assert 'follow_gps' in launch
    assert '/cmd_vel_auto' in launch


def test_navigation_launch_remaps_into_safety_layer():
    text = Path(__file__).resolve().parents[1].joinpath('launch', 'navigation.launch.py').read_text(encoding='utf-8')
    assert '/cmd_vel_auto' in text
    assert '/cmd_vel_raw' in text
    assert 'mapping.launch.py' not in text
