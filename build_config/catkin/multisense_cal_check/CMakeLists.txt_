cmake_minimum_required(VERSION 2.8.3)
project(multisense_cal_check)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS multisense_ros)

## Declare ROS messages and services

## Declare a catkin package
catkin_package(CATKIN_DEPENDS multisense_ros)

include_directories(include ${catkin_INCLUDE_DIRS})

## Install
install(PROGRAMS
  cal_check
  process_bags.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  distrib/MULTISENSE_SL_CHECK_CAL_32
  distrib/MULTISENSE_SL_CHECK_CAL_64
  distrib/run_MULTISENSE_SL_CHECK_CAL_32.sh
  distrib/run_MULTISENSE_SL_CHECK_CAL_64.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/distrib
)