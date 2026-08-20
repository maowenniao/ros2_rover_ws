"""full_system.launch.py

启动整个四轴差动无人小车系统，按顺序：
  0. CAN 总线（SocketCAN @ 250 kbps）
  1. Micro XRCE-DDS Agent（**Serial 模式** - USB 直连 Pixhawk 6C）
  2. robot_state_publisher（URDF 3D 模型）
  3. joint_state_bridge_node（/wheel_state → /joint_states）
  4. D435i 相机
  5. 差速车控制（cmd_vel → PX4 TrajectorySetpoint）
  6. ODrive CAN 桥接
  7. 轮速里程计
  8. RViz（可选）

启动方式：
    ros2 launch rover_bringup full_system.launch.py
    ros2 launch rover_bringup full_system.launch.py enable_rviz:=true
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup_can(context, *args, **kwargs):
    interface = LaunchConfiguration('can_interface').perform(context)
    bitrate = LaunchConfiguration('can_bitrate').perform(context)
    cmd = [
        'bash', '-c',
        f'sudo -n ip link set {interface} down type can bitrate {bitrate} 2>/dev/null; '
        f'sudo -n ip link set {interface} up type can bitrate {bitrate} 2>/dev/null; '
        f'echo "[can] {interface} @ {bitrate} bps OK"'
    ]
    return [ExecuteProcess(cmd=cmd, shell=False, output='screen')]


def _check_pixhawk(context, *args, **kwargs):
    device = LaunchConfiguration('pixhawk_device').perform(context)
    out = ExecuteProcess(
        cmd=['bash', '-c', f'ls -la {device} 2>/dev/null || echo "WARN: {device} 未找到，请先运行 setup_pixhawk_usb.sh"'],
        output='screen',
    )
    return [out]


def generate_launch_description() -> LaunchDescription:
    workspace = '/home/ai/Desktop/ros2_rover_ws'

    # ============== 参数声明 ==============
    declare_args = [
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_bitrate', default_value='250000'),
        DeclareLaunchArgument('enable_d435i', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('enable_agent', default_value='true'),
        DeclareLaunchArgument('pixhawk_device', default_value='/dev/pixhawk',
                              description='Pixhawk 6C 串口设备（udev 固定名）'),
        DeclareLaunchArgument('uxrce_baudrate', default_value='921600'),
        DeclareLaunchArgument('d435i_serial', default_value=''),
        DeclareLaunchArgument('enable_urdf', default_value='true'),
    ]

    # ============== CAN 总线配置 ==============
    can_setup = OpaqueFunction(function=_setup_can)
    pixhawk_check = OpaqueFunction(function=_check_pixhawk)

    # ============== 1. Micro XRCE-DDS Agent (Serial 模式) ==============
    uxrce_agent = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'MicroXRCEAgent serial -D '
            f'{LaunchConfiguration("pixhawk_device")} '
            f'-b {LaunchConfiguration("uxrce_baudrate")} '
            '-v 6'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_agent')),
    )

    # ============== 2. URDF + robot_state_publisher ==============
    urdf_xacro = os.path.join(
        FindPackageShare('rover_description').find('rover_description'),
        'urdf', 'rover.urdf.xacro'
    )
    robot_description = Command(['xacro ', urdf_xacro])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        condition=IfCondition(LaunchConfiguration('enable_urdf')),
    )

    # ============== 3. joint_state_bridge_node ==============
    joint_state_bridge = Node(
        package='rover_description',
        executable='joint_state_bridge_node',
        name='joint_state_bridge_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_urdf')),
    )

    # ============== 4. D435i 相机 ==============
    d435i_node = Node(
        package='d435i_launch',
        executable='d435i.launch.py',
        condition=IfCondition(LaunchConfiguration('enable_d435i')),
        name='d435i_launch',
        output='screen',
        parameters=[{
            'serial_no': LaunchConfiguration('d435i_serial'),
        }],
    )

    # ============== 5. 差速车控制节点 ==============
    diffdrive_cmdvel = Node(
        package='diffdrive_rover',
        executable='cmdvel_to_setpoint_node',
        name='cmdvel_to_setpoint_node',
        output='screen',
        parameters=[{
            'wheel_radius': 0.15,
            'wheel_base': 0.60,
            'track_width': 0.55,
            'max_linear_vel': 1.5,
            'max_angular_vel': 1.5,
            'setpoint_rate_hz': 20.0,
            'velocity_mode': True,
        }],
    )

    diffdrive_odom = Node(
        package='diffdrive_rover',
        executable='odometry_bridge_node',
        name='odometry_bridge_node',
        output='screen',
        parameters=[{
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'publish_tf': True,
        }],
    )

    # ============== 6. ODrive CAN 桥接 ==============
    odrive_can = Node(
        package='odrive_can_bridge',
        executable='odrive_can_node',
        name='odrive_can_node',
        output='screen',
        parameters=[
            os.path.join(workspace, 'src/odrive_can_bridge/config/'
                        'odrive_can_params.yaml'),
        ],
    )

    odrive_wheel_odom = Node(
        package='odrive_can_bridge',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[{
            'wheel_radius': 0.15,
            'track_width': 0.55,
            'frame_id': 'wheel_odom',
            'child_frame_id': 'base_link',
        }],
    )

    # ============== 7. RViz（可选）==============
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(workspace, 'src/rover_bringup/rviz/rover.rviz')],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return LaunchDescription([
        *declare_args,
        can_setup,
        pixhawk_check,
        TimerAction(period=0.5, actions=[uxrce_agent]),
        TimerAction(period=1.0, actions=[rsp_node]),
        TimerAction(period=1.0, actions=[joint_state_bridge]),
        TimerAction(period=1.5, actions=[d435i_node]),
        TimerAction(period=2.0, actions=[diffdrive_cmdvel]),
        TimerAction(period=2.0, actions=[diffdrive_odom]),
        TimerAction(period=2.5, actions=[odrive_can]),
        TimerAction(period=2.5, actions=[odrive_wheel_odom]),
        TimerAction(period=3.5, actions=[rviz_node]),
    ])