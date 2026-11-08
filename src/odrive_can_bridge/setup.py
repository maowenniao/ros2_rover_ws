from setuptools import setup
import os
from glob import glob

package_name = 'odrive_can_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    package_data={'odrive_can_bridge': ['templates/*.html']},
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Rover Developer',
    maintainer_email='you@example.com',
    description='ODrive V3.6 CAN ↔ ROS 2 桥接节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_can_node = odrive_can_bridge.odrive_can_node:main',
            'wheel_odometry_node = odrive_can_bridge.wheel_odometry_node:main',
            'odrive_velocity_control = odrive_can_bridge.odrive_velocity_control_node:main',
            'can_bus_test = odrive_can_bridge.can_bus_test:main',
            'web_controller = odrive_can_bridge.web_controller:main',
            'odrive_torque_diag = odrive_can_bridge.torque_diag:main',
            'odrive_quad_torque_diag = odrive_can_bridge.quad_torque_diag:main',
        ],
    },
)