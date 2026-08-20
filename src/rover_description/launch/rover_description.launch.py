"""rover_description.launch.py - 加载 URDF 并发布 robot_state + tf

启动方式：
    ros2 launch rover_description rover_description.launch.py
    ros2 launch rover_description rover_description.launch.py use_gui:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = 'rover_description'

    # URDF xacro → robot_description
    urdf_xacro = os.path.join(
        FindPackageShare(pkg).find(pkg), 'urdf', 'rover.urdf.xacro'
    )

    robot_description = Command(['xacro ', urdf_xacro])

    # robot_state_publisher：发布 /tf (base_link → 传感器 + 轮子)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # joint_state_publisher：发布关节状态（4 个轮子根据 /wheel_state 实时更新）
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=LaunchConfiguration('use_gui'),
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=LaunchConfiguration('use_gui'),
    ) if False else None  # use joint_state_publisher_gui 替代

    # 关节角度 → /wheel_state 桥接节点（在 full_system 中已包含）
    # 这里提供一个静态版本：发布固定角度（仅用于 RViz 显示模型）
    static_jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'rate': 20}],
        condition=LaunchConfiguration('use_gui'),
    ) if False else None

    declare_args = [
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='是否启动 joint_state_publisher_gui（带滑块）'),
    ]

    nodes = [rsp_node, jsp_gui_node]
    if static_jsp_node:
        nodes.append(static_jsp_node)
    if jsp_node:
        nodes.append(jsp_node)

    return LaunchDescription(declare_args + nodes)