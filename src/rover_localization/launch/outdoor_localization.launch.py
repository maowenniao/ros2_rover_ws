"""户外定位：已有的本地 EKF，再加上地图 EKF 和 navsat。

本地滤波仍读 ekf.yaml，室内那份行为不变。
map -> odom 由地图 EKF 发布。不要和 AMCL 或 slam_toolbox 一起开。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rover_localization')
    ekf_map = os.path.join(share, 'config', 'ekf_map.yaml')
    navsat = os.path.join(share, 'config', 'navsat.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'localization.launch.py')),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_map],
            remappings=[('odometry/filtered', '/odometry/global')],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat],
            remappings=[
                ('imu', '/px4/imu'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered'),
            ],
        ),
    ])
