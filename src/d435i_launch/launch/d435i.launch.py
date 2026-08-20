"""d435i.launch.py - Intel RealSense D435i 启动（带对齐点云）。

参数说明：
    depth_width/height/fps: 深度图分辨率
    color_width/height/fps: 彩色图分辨率
    enable_infra1/2: 红外 1/2 通道（IMU 来自 sensor_msgs/Imu）
    enable_gyro/accel: 启用 IMU 陀螺仪/加速度计
    align_depth.enable: 是否将深度图与彩色图对齐（影响点云质量）
    pointcloud.enable: 是否输出点云
    unite_imu_method: 0=None, 1=copy, 2=linear_interpolation
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 参数声明
    declare_args = [
        DeclareLaunchArgument('depth_width', default_value='640'),
        DeclareLaunchArgument('depth_height', default_value='480'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('color_fps', default_value='30'),
        DeclareLaunchArgument('enable_infra1', default_value='true'),
        DeclareLaunchArgument('enable_infra2', default_value='true'),
        DeclareLaunchArgument('enable_gyro', default_value='true'),
        DeclareLaunchArgument('enable_accel', default_value='true'),
        DeclareLaunchArgument('enable_pointcloud', default_value='true'),
        DeclareLaunchArgument('enable_align_depth', default_value='true'),
        DeclareLaunchArgument('unite_imu_method', default_value='1'),
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('camera_name', default_value='d435i'),
    ]

    # 构建 realsense2_camera 节点
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[{
            'depth_module.depth_profile':
                str(LaunchConfiguration('depth_width')) + 'x' +
                str(LaunchConfiguration('depth_height')) + 'x' +
                str(LaunchConfiguration('depth_fps')),
            'rgb_camera.color_profile':
                str(LaunchConfiguration('color_width')) + 'x' +
                str(LaunchConfiguration('color_height')) + 'x' +
                str(LaunchConfiguration('color_fps')),
            'infra1.enable': LaunchConfiguration('enable_infra1'),
            'infra2.enable': LaunchConfiguration('enable_infra2'),
            'gyro.enable': LaunchConfiguration('enable_gyro'),
            'accel.enable': LaunchConfiguration('enable_accel'),
            'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
            'align_depth.enable': LaunchConfiguration('enable_align_depth'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'serial_no': LaunchConfiguration('serial_no'),
            'device_type': 'd435i',
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(declare_args + [realsense_node])