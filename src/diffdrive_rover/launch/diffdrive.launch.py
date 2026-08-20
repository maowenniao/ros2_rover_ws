"""diffdrive.launch.py - 启动差速车控制 + 里程计桥接节点。"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = 'diffdrive_rover'
    config_dir = os.path.join(
        os.path.pkg_share(pkg), 'config'
    ) if False else '/home/ai/Desktop/ros2_rover_ws/src/diffdrive_rover/config'

    params_file = os.path.join(config_dir, 'diffdrive_params.yaml')

    cmdvel_node = Node(
        package=pkg,
        executable='cmdvel_to_setpoint_node',
        name='cmdvel_to_setpoint_node',
        output='screen',
        parameters=[params_file],
    )

    odom_node = Node(
        package=pkg,
        executable='odometry_bridge_node',
        name='odometry_bridge_node',
        output='screen',
        parameters=[
            {'frame_id': 'odom',
             'child_frame_id': 'base_link',
             'publish_tf': True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='差速车参数文件',
        ),
        cmdvel_node,
        odom_node,
    ])