cmake_minimum_required(VERSION 3.0.2)
project(duckiebot_slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  tf
  duckietown_msgs
)

catkin_package()

install(PROGRAMS
  src/odometry_node.py
  src/slam_node.py
  src/fusion_node.py
  src/wheel_control_node.py
  src/camera_reader_node.py
  src/wheel_encoder_reader_node.py
  src/twist_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
