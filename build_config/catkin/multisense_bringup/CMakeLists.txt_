cmake_minimum_required(VERSION 2.8.3)
project(multisense_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS multisense_ros)

## Declare ROS messages and services

## Declare a catkin package
catkin_package(CATKIN_DEPENDS multisense_ros)

include_directories(include ${catkin_INCLUDE_DIRS})

## Install ##
# Install Gazebo launch files
install(FILES
  rviz_config.rviz
  multisense.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
