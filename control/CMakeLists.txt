# ---------------------------------------------------------------------------- #
# -------------------- Path Control Module (PCM) - ARAV ---------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

cmake_minimum_required (VERSION 3.0.2)
project (control)
find_package (
  catkin REQUIRED
  gazebo_msgs
  geometry_msgs
  std_msgs
  tf
  rospy 
)
include_directories (include ${catkin_INCLUDE_DIRS})

# ---------------------------------------------------------------------------- #
# -------------------- Path Control Module (PCM) - ARAV ---------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

