from setuptools import setup
import os
from glob import glob

package_name = 'rover_mission'

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
    maintainer='maowenniao',
    maintainer_email='maowenniao@163.com',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_mission_node = rover_mission.waypoint_mission_node:main',
            'safety_monitor_node = rover_mission.safety_monitor_node:main',
        ],
    },
)
