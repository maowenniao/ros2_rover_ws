from setuptools import setup
import os
from glob import glob

package_name = 'rover_description'

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
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz') + glob('config/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Developer',
    maintainer_email='you@example.com',
    description='四轴差动无人小车的 3D 模型描述包（URDF）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_bridge_node = rover_description.joint_state_bridge_node:main',
        ],
    },
)