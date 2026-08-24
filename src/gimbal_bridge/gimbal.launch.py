#!/usr/bin/env python3
"""
gimbal.launch.py  --  bridge + plot in one shot.

Run (no colcon build needed -- ros2 launch accepts a file path):
    ros2 launch ~/gimbalarm_ws/src/gimbal_bridge/gimbal.launch.py
    ros2 launch ~/gimbalarm_ws/src/gimbal_bridge/gimbal.launch.py port:=/dev/ttyUSB0

Starts:
  - gimbal_bridge.py  (serial <-> /gimbal/* topics, commands via /gimbal/key)
  - plotjuggler       (then: Streaming->ROS2 Topic Subscriber->Start->tick /gimbal/*)

In launch mode stdin is not a terminal, so send menu commands by topic:
    ros2 topic pub --once /gimbal/key std_msgs/msg/String "{data: C}"   # hold
    ros2 topic pub --once /gimbal/key std_msgs/msg/String "{data: T}"   # telemetry on
    ros2 topic pub --once /gimbal/key std_msgs/msg/String "{data: B}"   # toggle ESO

Needs: pyserial, and PlotJuggler (sudo apt install ros-jazzy-plotjuggler-ros).
(rqt_plot is avoided: it needs matplotlib/pyqtgraph which break under NumPy 2.x here.)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    bridge = os.path.join(here, "gimbal_bridge.py")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="ESP32 serial device"),
        ExecuteProcess(
            cmd=["python3", bridge, port],
            output="screen",
            name="gimbal_bridge",
        ),
        # PlotJuggler (C++ Qt, no python/numpy deps). In the GUI:
        #   Streaming -> "ROS2 Topic Subscriber" -> Start -> tick /gimbal/* -> OK,
        #   then drag /gimbal/err and /gimbal/dob into the plot.
        ExecuteProcess(
            cmd=["ros2", "run", "plotjuggler", "plotjuggler"],
            output="screen",
            name="plotjuggler",
        ),
    ])
