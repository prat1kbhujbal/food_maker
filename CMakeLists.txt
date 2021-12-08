cmake_minimum_required(VERSION 3.0.2)
project(food_maker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES food_maker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Add models folder to path
install(DIRECTORY models 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)