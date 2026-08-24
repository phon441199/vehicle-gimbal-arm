#!/usr/bin/env bash
# build + launch fake-hw RViz sim
set -e
cd ~/gimbalarm_ws
colcon build --packages-select gimbal_arm_controller gimbalarm_description
source install/setup.bash
ros2 launch gimbal_arm_controller sim.launch.py
