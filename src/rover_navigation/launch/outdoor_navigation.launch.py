"""户外 Nav2：滚动代价地图，不加载先前保存的地图。经纬度航点默认不起步。"""
import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_description.geometry import load_geometry
from rover_navigation.footprint import limit_geometry, materialize_nav_params


def _nodes(context, *args, **kwargs):
    share = get_package_share_directory('rover_navigation')
    template = Path(os.path.join(share, 'config', 'nav2_outdoor_params.yaml')).read_text(encoding='utf-8')
    geometry = limit_geometry(
        load_geometry(),
        float(LaunchConfiguration('cruise_linear').perform(context)),
        float(LaunchConfiguration('cruise_angular').perform(context)),
    )
    rendered = materialize_nav_params(template, geometry)
    params_path = Path(tempfile.gettempdir()) / 'rover_nav2_outdoor_params.yaml'
    params_path.write_text(rendered, encoding='utf-8')
    params = str(params_path)
    raw = ('cmd_vel', '/cmd_vel_raw')
    smooth = [('cmd_vel', '/cmd_vel_raw'), ('cmd_vel_smoothed', '/cmd_vel_auto')]
    common = {'output': 'screen', 'parameters': [params]}
    waypoints = LaunchConfiguration('waypoints_file').perform(context)
    return [
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
                    'planner_server', 'controller_server', 'behavior_server',
                    'bt_navigator', 'waypoint_follower', 'velocity_smoother',
                ],
            }],
        ),
        Node(
            package='rover_navigation',
            executable='gps_waypoint_node',
            name='gps_waypoint_node',
            output='screen',
            parameters=[{
                'waypoints_file': waypoints,
                'autostart': LaunchConfiguration('follow_gps'),
                'start_delay_sec': 8.0,
            }],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('rover_navigation')
    default_waypoints = os.path.join(share, 'config', 'gps_waypoints.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('follow_gps', default_value='false'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('cruise_linear', default_value='0.30'),
        DeclareLaunchArgument('cruise_angular', default_value='0.50'),
        OpaqueFunction(function=_nodes),
    ])
