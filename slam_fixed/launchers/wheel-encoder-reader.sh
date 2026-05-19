#!/bin/bash
source /environment.sh
source /code/devel/setup.bash
source /code/catkin_ws/devel/setup.bash
export VEHICLE_NAME=${VEHICLE_NAME:-duckiebot21}
export ROS_MASTER_URI=http://duckiebot21.local:11311
dt-launchfile-init
rosrun duckiebot_slam wheel_encoder_reader_node.py
dt-launchfile-join
