"""加载 URDF。尺寸来自 rover_geometry.yaml，可用 use_gui 打开关节滑块。"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from rover_description.geometry import load_geometry


def generate_launch_description() -> LaunchDescription:
    geometry = load_geometry()
    urdf = os.path.join(FindPackageShare('rover_description').find('rover_description'), 'urdf', 'rover.urdf.xacro')
    robot = Command([
        'xacro ', urdf,
        ' wheel_radius:=', str(geometry['wheel_radius']),
        ' track_width:=', str(geometry['track_width']),
        ' wheel_base:=', str(geometry['wheel_base']),
        ' body_length:=', str(geometry['wheel_base']),
        ' body_width:=', str(geometry['track_width']),
        ' gps_x:=', str(geometry['gps_x']),
        ' gps_y:=', str(geometry['gps_y']),
        ' gps_z:=', str(geometry['gps_z']),
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot, 'use_sim_time': False}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gui')),
        ),
    ])
