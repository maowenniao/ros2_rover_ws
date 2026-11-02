"""Nav2 导航：地图 + AMCL + 规划控制。不启动 slam_toolbox。

控制器输出到 /cmd_vel_raw，平滑后到 /cmd_vel_nav，再由安全层写 /cmd_vel。
"""
import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_description.geometry import load_geometry
from rover_navigation.footprint import materialize_nav_params


def _nodes(context, *args, **kwargs):
    share = get_package_share_directory('rover_navigation')
    template = Path(os.path.join(share, 'config', 'nav2_params.yaml')).read_text(encoding='utf-8')
    rendered = materialize_nav_params(template, load_geometry())
    params_path = Path(tempfile.gettempdir()) / 'rover_nav2_params.yaml'
    params_path.write_text(rendered, encoding='utf-8')
    params = str(params_path)
    map_yaml = LaunchConfiguration('map').perform(context)
    raw = ('cmd_vel', '/cmd_vel_raw')
    smooth = [('cmd_vel', '/cmd_vel_raw'), ('cmd_vel_smoothed', '/cmd_vel_nav')]
    common = {'output': 'screen', 'parameters': [params]}
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': map_yaml}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', **common),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params], remappings=[raw], output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[params], remappings=[raw], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[params], remappings=smooth, output='screen'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server', 'amcl', 'planner_server', 'controller_server',
                    'behavior_server', 'bt_navigator', 'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
    ]


def generate_launch_description():
    default_map = os.path.join(
        get_package_share_directory('rover_navigation'), 'maps', 'rover_warehouse.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        OpaqueFunction(function=_nodes),
    ])
