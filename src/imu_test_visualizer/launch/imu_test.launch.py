import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('imu_test_visualizer'),
        'config',
        'imu_rviz.rviz'
    )

    # micro-ROS agent (native)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen',
    )

    # world → imu_link 정적 변환
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'world', '--child-frame-id', 'imu_link'],
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        micro_ros_agent,
        static_tf,
        rviz2,
    ])
