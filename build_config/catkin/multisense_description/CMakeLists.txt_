cmake_minimum_required(VERSION 2.8.3)
project(multisense_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## Declare ROS messages and services

find_package(catkin REQUIRED)

## Declare a catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

## Install
install(DIRECTORY
  urdf meshes materials robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install below for gazebo to find this model through GAZEBO_MODEL_PATH.
install(FILES
  model.config
  dummy.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
