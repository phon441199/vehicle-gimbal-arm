#!/usr/bin/env bash
# send a goal to pose_controller (run in a 2nd terminal)
#   ./goto.sh standby            -> named pose
#   ./goto.sh receive
#   ./goto.sh slosh
#   ./goto.sh 0.15 0.0 0.6       -> cup-center cartesian target [m]
cd ~/gimbalarm_ws
export ROS_DOMAIN_ID=5
source install/setup.bash
if [ "$#" -eq 3 ]; then
  ros2 topic pub --once /cup_target geometry_msgs/PointStamped \
    "{point: {x: $1, y: $2, z: $3}}"
else
  ros2 topic pub --once /goto_pose std_msgs/String "{data: ${1:-standby}}"
fi
