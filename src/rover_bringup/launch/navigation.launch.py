"""导航：底盘 + 相机 + EKF + AMCL + Nav2。不开 slam_toolbox，避免两份 map -> odom。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_share = get_package_share_directory('rover_navigation')
    default_map = os.path.join(nav_share, 'maps', 'rover_warehouse.yaml')

    def include(package, launch_file, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package), 'launch', launch_file)),
            launch_arguments=args or {},
        )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('enable_agent', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        include('rover_bringup', 'full_system.launch.py', {
            'enable_safety': 'true',
            'enable_d435i': 'true',
            'publish_odom_tf': 'false',
            'enable_agent': LaunchConfiguration('enable_agent'),
            'enable_rviz': LaunchConfiguration('enable_rviz'),
        }.items()),
        include('rover_mapping', 'depth_to_scan.launch.py'),
        include('rover_localization', 'localization.launch.py'),
        include('rover_navigation', 'navigation.launch.py', {
            'map': LaunchConfiguration('map'),
        }.items()),
    ])
