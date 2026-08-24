import os
from glob import glob
from setuptools import setup

package_name = 'gimbalarm_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phon419',
    maintainer_email='mecanic419@gmail.com',
    description='GimbalArm 5-DOF URDF/xacro description for RViz, IK, kinematic sim.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
