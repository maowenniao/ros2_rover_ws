"""只启动 slam_toolbox。map -> odom 由它发布，不要同时开 AMCL。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('rover_mapping'),
        'config', 'mapper_params_online_async.yaml')
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params],
        ),
    ])
