#!/usr/bin/env bash
# GimbalArm tkinter command console, standalone (launch already includes it via gui:=true).
#   ./arm_gui.sh        (run `jazzy` first)
cd ~/gimbalarm_ws
export ROS_DOMAIN_ID=5
source install/setup.bash
exec ros2 run gimbal_arm_controller arm_gui
