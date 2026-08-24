from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gimbal_arm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phon419',
    maintainer_email='phon419@todo.com',
    description='5-axis robot arm controller using micro-ROS and Teensy 4.1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = gimbal_arm_controller.arm_controller_node:main',
            'pose_controller = gimbal_arm_controller.pose_controller_node:main',
            'arm_gui = gimbal_arm_controller.arm_gui_node:main',
        ],
    },
)
