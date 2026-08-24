import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('gimbal_arm_controller'),
        'config',
        'arm_params.yaml'
    )

    arm_controller_node = Node(
        package='gimbal_arm_controller',
        executable='arm_controller',
        name='arm_controller',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        arm_controller_node,
    ])
