"""底盘 + 可选安全层 + PX4 链路 + 模型。

/cmd_vel → odrive_velocity_control → /wheel_cmd → odrive_can_node
→ /wheel_state → wheel_odometry（唯一的 odom → base_link）。

PX4 只用于解锁、急停和飞控状态，不接收速度设定点。
enable_safety:=true 时，安全层把 /cmd_vel_nav 限速后写到 /cmd_vel。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from rover_description.geometry import load_geometry


def _setup_can(context, *args, **kwargs):
    interface = LaunchConfiguration('can_interface').perform(context)
    bitrate = LaunchConfiguration('can_bitrate').perform(context)
    return [ExecuteProcess(
        cmd=['bash', '-c',
             f'sudo -n ip link set {interface} down 2>/dev/null; '
             f'sudo -n ip link set {interface} up type can bitrate {bitrate} 2>/dev/null; '
             f'echo "[can] {interface} @ {bitrate}"'],
        output='screen',
    )]


def generate_launch_description() -> LaunchDescription:
    geometry = load_geometry()
    description_share = get_package_share_directory('rover_description')
    bringup_share = get_package_share_directory('rover_bringup')
    odrive_share = get_package_share_directory('odrive_can_bridge')
    urdf = os.path.join(description_share, 'urdf', 'rover.urdf.xacro')
    robot = Command([
        'xacro ', urdf,
        ' wheel_radius:=', str(geometry['wheel_radius']),
        ' track_width:=', str(geometry['track_width']),
        ' wheel_base:=', str(geometry['wheel_base']),
        ' body_length:=', str(geometry['wheel_base']),
        ' body_width:=', str(geometry['track_width']),
    ])
    rviz_config = os.path.join(bringup_share, 'rviz', 'rover.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_bitrate', default_value='250000'),
        DeclareLaunchArgument('enable_d435i', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('enable_agent', default_value='true'),
        DeclareLaunchArgument('enable_safety', default_value='false'),
        DeclareLaunchArgument('enable_web', default_value='false'),
        DeclareLaunchArgument('pixhawk_device', default_value='/dev/pixhawk'),
        DeclareLaunchArgument('uxrce_baudrate', default_value='921600'),
        DeclareLaunchArgument('d435i_serial', default_value="''"),
        DeclareLaunchArgument('publish_odom_tf', default_value='true',
                              description='建图或导航时应为 false，改由 EKF 发布 odom'),
        OpaqueFunction(function=_setup_can),
        ExecuteProcess(
            cmd=[
                'MicroXRCEAgent', 'serial',
                '-D', LaunchConfiguration('pixhawk_device'),
                '-b', LaunchConfiguration('uxrce_baudrate'),
                '-v', '6',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_agent')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot, 'use_sim_time': False}],
        ),
        Node(
            package='rover_description',
            executable='joint_state_bridge_node',
            name='joint_state_bridge_node',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(odrive_share, 'launch', 'odrive_can.launch.py')),
            launch_arguments={
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('diffdrive_rover'),
                             'launch', 'safety_cmdvel.launch.py')),
            condition=IfCondition(LaunchConfiguration('enable_safety')),
        ),
        Node(
            package='rover_bringup',
            executable='estop_action_node',
            name='estop_action',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('d435i_launch'),
                             'launch', 'd435i.launch.py')),
            launch_arguments={'serial_no': LaunchConfiguration('d435i_serial')}.items(),
            condition=IfCondition(LaunchConfiguration('enable_d435i')),
        ),
        Node(
            package='odrive_can_bridge',
            executable='web_controller',
            name='web_controller',
            output='screen',
            parameters=[os.path.join(description_share, 'config', 'rover_geometry.yaml')],
            condition=IfCondition(LaunchConfiguration('enable_web')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
            output='screen',
        ),
    ])
