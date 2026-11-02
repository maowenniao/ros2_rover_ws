"""/cmd_vel_nav 经限速后发布 /cmd_vel。没有扫描时默认停车。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text: str) -> bool:
    return text.lower() in ('1', 'true', 'yes', 'on')


def _launch(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    return [Node(
        package='diffdrive_rover',
        executable='safety_cmdvel_node',
        name='safety_cmdvel_node',
        output='screen',
        parameters=[{
            'stop_distance': float(arg('stop_distance')),
            'slow_distance': float(arg('slow_distance')),
            'require_scan': _as_bool(arg('require_scan')),
            'cmd_timeout': float(arg('cmd_timeout')),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stop_distance', default_value='0.45'),
        DeclareLaunchArgument('slow_distance', default_value='1.20'),
        DeclareLaunchArgument('require_scan', default_value='true'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        OpaqueFunction(function=_launch),
    ])
