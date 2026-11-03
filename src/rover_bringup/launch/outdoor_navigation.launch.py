"""户外：底盘 + 相机 + 双 EKF + navsat + Nav2。不开建图，不开 AMCL。

室内建图和室内导航不会启动 navsat，也不会等 GPS。本文件只在室外使用。
follow_gps:=true 才会按经纬度出发，并且要先有 /gps/fix。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_waypoints = os.path.join(
        get_package_share_directory('rover_navigation'), 'config', 'gps_waypoints.yaml')

    def include(package, launch_file, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package), 'launch', launch_file)),
            launch_arguments=args or {},
        )

    return LaunchDescription([
        DeclareLaunchArgument('follow_gps', default_value='false'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('cruise_linear', default_value='0.30'),
        DeclareLaunchArgument('cruise_angular', default_value='0.50'),
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
        include('rover_localization', 'outdoor_localization.launch.py'),
        include('rover_navigation', 'outdoor_navigation.launch.py', {
            'follow_gps': LaunchConfiguration('follow_gps'),
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'cruise_linear': LaunchConfiguration('cruise_linear'),
            'cruise_angular': LaunchConfiguration('cruise_angular'),
        }.items()),
        include('diffdrive_rover', 'rc_command.launch.py'),
    ])
