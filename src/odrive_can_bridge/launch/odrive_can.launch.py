"""底盘链路：/cmd_vel → /wheel_cmd → ODrive CAN → /wheel_state → /wheel_odom。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    geometry = os.path.join(
        get_package_share_directory('rover_description'), 'config', 'rover_geometry.yaml')
    odrive_params = os.path.join(
        get_package_share_directory('odrive_can_bridge'), 'config', 'odrive_can_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file', default_value=geometry),
        DeclareLaunchArgument('odrive_params', default_value=odrive_params),
        Node(
            package='odrive_can_bridge',
            executable='odrive_velocity_control',
            name='odrive_velocity_control',
            output='screen',
            parameters=[LaunchConfiguration('geometry_file')],
        ),
        Node(
            package='odrive_can_bridge',
            executable='odrive_can_node',
            name='odrive_can_node',
            output='screen',
            parameters=[
                LaunchConfiguration('geometry_file'),
                LaunchConfiguration('odrive_params'),
            ],
        ),
        Node(
            package='odrive_can_bridge',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            parameters=[LaunchConfiguration('geometry_file')],
        ),
    ])
