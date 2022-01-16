# ---------------------------------------------------------------------------- #
# ---------------------- Self-Driving Simulator - ARAV ----------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

cmake_minimum_required (VERSION 3.0.2)
project (simulator)
find_package (
  catkin REQUIRED
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  joint_state_publisher
  robot_state_publisher
  rqt_robot_steering
  rviz
  sensor_msgs
  std_msgs
  tf2_ros
  xacro
  image_transport
  cv_bridge
  roscpp
  rospy 
)
find_package (OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories (include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# ---------------------------------------------------------------------------- #
# ---------------------- Self-Driving Simulator - ARAV ----------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

