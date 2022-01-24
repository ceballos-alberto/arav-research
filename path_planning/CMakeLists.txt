
# ---------------------------------------------------------------------------- #
# -------------------- Path Planning Module (EPM) - ARAV --------------------- #
# ---------------------------------------------------------------------------- #
# ------------------------- Author : Joan Bessa Sanz ------------------------- #
# ------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ------------- #
# ------------ (c) Copyright 2021. Joan Bessa. All Rights Reserved ----------- #
# ---------------------------------------------------------------------------- #

cmake_minimum_required (VERSION 3.0.2)
project (path_planning)

set (CMAKE_CXX_STANDARD 11)

find_package(octomap REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)

find_package (
  catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros  
  roscpp
  std_msgs
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
	${OMPL_INCLUDE_DIRS}
	${fcl_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)

link_libraries(
	${OCTOMAP_LIBRARIES}
	${fcl_LIBRARY_DIRS}
	${OMPL_LIBRARY_DIRS}
)

SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/code)
				
add_executable (path_planning_node code/path_planning_node.cpp)

target_link_libraries( path_planning_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${fcl_LIBRARIES}
   ${OMPL_LIBRARIES}
   fcl
)

# ---------------------------------------------------------------------------- #
# -------------------- Path Planning Module (EPM) - ARAV --------------------- #
# ---------------------------------------------------------------------------- #
# ------------------------- Author : Joan Bessa Sanz ------------------------- #
# ------------- E-mail : joan.bessa-sanz@student.isae-supaero.fr ------------- #
# ------------ (c) Copyright 2021. Joan Bessa. All Rights Reserved ----------- #
# ---------------------------------------------------------------------------- #

