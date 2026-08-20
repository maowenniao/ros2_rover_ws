from setuptools import setup
import os
from glob import glob

package_name = 'diffdrive_rover'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Developer',
    maintainer_email='you@example.com',
    description='四轮差速小车控制包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdvel_to_setpoint_node = diffdrive_rover.cmdvel_to_setpoint_node:main',
            'odometry_bridge_node = diffdrive_rover.odometry_bridge_node:main',
            'safety_cmdvel_node = diffdrive_rover.safety_cmdvel_node:main',
        ],
    },
)