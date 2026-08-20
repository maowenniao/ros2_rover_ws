"""启动导航速度安全层。

Nav2/遥控器发布到 /cmd_vel_nav；安全层输出 /cmd_vel，供底盘桥接节点使用。
LaserScan 可以来自真实雷达或深度相机转换节点。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stop_distance', default_value='0.45'),
        DeclareLaunchArgument('slow_distance', default_value='1.20'),
        DeclareLaunchArgument('require_scan', default_value='true'),
        Node(
            package='diffdrive_rover',
            executable='safety_cmdvel_node',
            name='safety_cmdvel_node',
            output='screen',
            parameters=[{
                'stop_distance': LaunchConfiguration('stop_distance'),
                'slow_distance': LaunchConfiguration('slow_distance'),
                'require_scan': LaunchConfiguration('require_scan'),
            }],
        ),
    ])
