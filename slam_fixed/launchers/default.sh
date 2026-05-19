#!/bin/bash
source /environment.sh
# Use VEHICLE_NAME from environment if set, otherwise auto-detect from hostname
export VEHICLE_NAME=${VEHICLE_NAME:-$(hostname)}

echo "Building catkin workspace..."
mkdir -p /code/catkin_ws/src
cd /code/catkin_ws

/bin/bash -c "
    source /opt/ros/noetic/setup.bash && \
    source /code/devel/setup.bash && \
    catkin_make -j1 && \
    echo 'catkin_make SUCCESS'
"

if [ $? -ne 0 ]; then
    echo "ERROR: catkin_make failed"
    exit 1
fi

source /code/devel/setup.bash
source /code/catkin_ws/devel/setup.bash
export VEHICLE_NAME=${VEHICLE_NAME:-$(hostname)}
export ROS_MASTER_URI=http://${VEHICLE_NAME}.local:11311

dt-launchfile-init

echo "Starting nodes..."
# Taak 1 — Odometry
rosrun duckiebot_slam odometry_node.py &
sleep 1
# Taak 3 — Fusion (EKF)
rosrun duckiebot_slam fusion_node.py &
sleep 1
# Taak 2 — SLAM
rosrun duckiebot_slam slam_node.py &
sleep 1
# Path publisher voor RViz
rosrun duckiebot_slam path_publisher_node.py &

echo "All nodes started!"
echo "  Odometry  -> /${VEHICLE_NAME}/odometry/pose"
echo "  Fusion    -> /${VEHICLE_NAME}/fusion_node/pose"
echo "  SLAM map  -> /${VEHICLE_NAME}/slam_node/map/compressed"
echo "  Features  -> /${VEHICLE_NAME}/slam_node/features/compressed"
echo "  Path      -> /${VEHICLE_NAME}/odometry/path"

dt-launchfile-join
