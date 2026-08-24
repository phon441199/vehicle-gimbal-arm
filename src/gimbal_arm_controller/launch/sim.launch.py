"""Unified bringup: SAME command path for sim and real hardware (fake_hw flag).

  SIM  (no motors):  ros2 launch gimbal_arm_controller sim.launch.py
      pose_controller publishes /joint_states itself -> RViz animates the exact
      trajectory it WOULD send to HW.  Test here first, safely.

  HW   (real arm):   ros2 launch gimbal_arm_controller sim.launch.py fake_hw:=false
      adds the micro-ROS agent; the MCU publishes /joint_states, pose_controller
      streams the SAME /joint_commands to it.  Drive both with goto.sh / /goto_pose.

      ros2 launch ... sim.launch.py fake_hw:=false dev:=/dev/ttyUSB1   # other port

Workflow: validate a pose/trajectory in SIM (limits, kinematics, looks right) ->
rerun the identical command with fake_hw:=false on HW.  NOTE: sim cannot catch a
wrong AK40_DIR sign (that lives in firmware) -> verify direction once on HW with a
small nudge.  Also /joint_states from the MCU is BEST_EFFORT; if the HW arm stays
at zero in RViz, set the firmware publisher back to reliable (see arm_microros).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('gimbalarm_description')
    ctrl_share = get_package_share_directory('gimbal_arm_controller')
    xacro_file = os.path.join(desc_share, 'urdf', 'gimbalarm.urdf.xacro')
    rviz_file = os.path.join(desc_share, 'rviz', 'gimbalarm.rviz')
    params = os.path.join(ctrl_share, 'config', 'arm_params.yaml')

    rviz = LaunchConfiguration('rviz')
    fake_hw = LaunchConfiguration('fake_hw')
    dev = LaunchConfiguration('dev')
    baud = LaunchConfiguration('baud')
    gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '5'),   # MCU/agent = domain 5

        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('fake_hw', default_value='true',
                              description='true=RViz sim (no motors); false=real HW'),
        DeclareLaunchArgument('dev', default_value='/dev/ttyACM0'),   # Teensy 4.1 = native USB CDC (ttyACM*); ESP32 was ttyUSB*
        DeclareLaunchArgument('baud', default_value='921600'),   # match firmware default_transport.cpp
        DeclareLaunchArgument('gui', default_value='true',
                              description='true=open tkinter command console'),

        # real HW only: micro-ROS agent (serial transport to the ESP32)
        Node(package='micro_ros_agent', executable='micro_ros_agent',
             arguments=['serial', '--dev', dev, '-b', baud],
             output='screen',
             condition=UnlessCondition(fake_hw)),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),

        Node(package='gimbal_arm_controller', executable='pose_controller',
             name='pose_controller', output='screen',
             parameters=[params, {'fake_hw': fake_hw}]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_file], output='screen',
             condition=IfCondition(rviz)),

        # tkinter command console (publishes /arm_enable, /joint_commands,
        # /level_enable, /set_kp, /set_wrist_kp; shows /joint_states feedback)
        Node(package='gimbal_arm_controller', executable='arm_gui',
             name='arm_gui', output='screen',
             condition=IfCondition(gui)),
    ])
