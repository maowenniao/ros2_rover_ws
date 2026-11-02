"""遥控器仲裁和 PX4 状态桥。几何限幅与底盘共用 rover_geometry.yaml。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('diffdrive_rover')
    geometry = os.path.join(
        get_package_share_directory('rover_description'), 'config', 'rover_geometry.yaml')
    rc_params = os.path.join(share, 'config', 'rc_params.yaml')
    return LaunchDescription([
        Node(
            package='diffdrive_rover',
            executable='rc_command_node',
            name='rc_command_node',
            output='screen',
            parameters=[geometry, rc_params],
        ),
        Node(
            package='diffdrive_rover',
            executable='px4_state_node',
            name='px4_state_node',
            output='screen',
            parameters=[rc_params],
        ),
    ])
