"""可选：只把 /cmd_vel 送给 PX4。不发布 /wheel_cmd，也不占用电机。

底盘请用 odrive_can_bridge 的 launch。这里的里程计默认不发 TF，避免和轮速里程计抢 base_link。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('diffdrive_rover')
    geometry = os.path.join(
        get_package_share_directory('rover_description'), 'config', 'rover_geometry.yaml')
    params = os.path.join(share, 'config', 'diffdrive_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('publish_tf', default_value='false'),
        Node(
            package='diffdrive_rover',
            executable='cmdvel_to_setpoint_node',
            name='cmdvel_to_setpoint_node',
            output='screen',
            parameters=[geometry, params],
        ),
        Node(
            package='diffdrive_rover',
            executable='odometry_bridge_node',
            name='odometry_bridge_node',
            output='screen',
            parameters=[{
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'publish_tf': LaunchConfiguration('publish_tf'),
            }],
        ),
    ])
