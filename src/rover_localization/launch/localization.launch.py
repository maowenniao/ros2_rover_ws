"""EKF：融合 /wheel_odom 与 D435i IMU，发布唯一的 odom -> base_link。

AMCL 在导航启动里，不要和 slam_toolbox 同时开。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('rover_localization'), 'config', 'ekf.yaml')
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params],
            remappings=[('odometry/filtered', '/odom')],
        ),
    ])
