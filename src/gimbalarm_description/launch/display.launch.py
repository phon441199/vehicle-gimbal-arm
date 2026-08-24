import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('gimbalarm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'gimbalarm.urdf.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'gimbalarm.rviz')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='launch RViz'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),

        # sliders publish /joint_states for kinematic preview (no hardware)
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_file], output='screen',
             condition=IfCondition(rviz)),
    ])
