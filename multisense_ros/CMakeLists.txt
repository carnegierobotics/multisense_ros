cmake_minimum_required(VERSION 2.8.3)
project(multisense_ros)

if(NOT CMAKE_CXX_STANDARD)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 11)
    # Default to C++17
    set(CMAKE_CXX_STANDARD 17)
  else()
    # Default to C++14
    set(CMAKE_CXX_STANDARD 14)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Werror)
endif()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        genmsg
                                        tf2
                                        tf2_ros
                                        tf2_geometry_msgs
                                        multisense_lib
                                        rosbag
                                        sensor_msgs
                                        geometry_msgs
                                        image_transport
                                        angles
                                        cv_bridge
                                        dynamic_reconfigure
                                        image_geometry
                                        message_generation
                                        diagnostic_updater)

find_library(LIBTURBOJPEG_LIBRARIES NAMES "libturbojpeg.so.0" "libturbojpeg.so.1")


generate_dynamic_reconfigure_options(cfg/multisense.cfg)

add_message_files(DIRECTORY msg
                  FILES
                  ChannelStatistics.msg
                  DeviceInfo.msg
                  RawCamConfig.msg
                  RawImuData.msg
                  RawLidarData.msg
                  RawCamCal.msg
                  RawCamData.msg
                  RawLidarCal.msg
                  Histogram.msg
                  DeviceStatus.msg
                  StampedPps.msg
                  PtpStatus.msg)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(INCLUDE_DIRS   include
               CATKIN_DEPENDS roscpp
                              std_msgs
                              genmsg
                              tf2
                              tf2_ros
                              tf2_geometry_msgs
                              multisense_lib
                              rosbag
                              sensor_msgs
                              angles
                              cv_bridge
                              dynamic_reconfigure
                              image_geometry
                              image_transport
                              message_runtime
                              message_generation
                              diagnostic_updater
               LIBRARIES      ${PROJECT_NAME})

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${multisense_lib_INCLUDE_DIRS})

## Multisense Library


add_library(${PROJECT_NAME} src/camera.cpp
                            src/camera_utilities.cpp
                            src/imu.cpp
                            src/laser.cpp
                            src/pps.cpp
                            src/point_cloud_utilities.cpp
                            src/status.cpp
                            src/statistics.cpp
                            src/reconfigure.cpp
                            src/ground_surface_utilities.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${multisense_lib_LIBRARIES})


set_target_properties(${PROJECT_NAME}
  PROPERTIES LINK_FLAGS "-L${PROJECT_SOURCE_DIR}/../multisense_lib/sensor_api/bin/")
set_target_properties(${PROJECT_NAME}
  PROPERTIES COMPILE_FLAGS "-I${PROJECT_SOURCE_DIR}/../multisense_lib/sensor_api/source/LibMultiSense")

## ROS Driver

add_executable(ros_driver src/ros_driver.cpp)
target_link_libraries(ros_driver ${PROJECT_NAME}
                                 ${LIBTURBOJPEG_LIBRARIES})
set_target_properties(ros_driver
  PROPERTIES COMPILE_FLAGS "-I${PROJECT_SOURCE_DIR}/../multisense_lib/sensor_api/source/LibMultiSense")

add_dependencies(ros_driver ${PROJECT_NAME}_generate_messages_cpp)


## Raw Snapshot

add_executable(raw_snapshot src/raw_snapshot.cpp)
add_dependencies(raw_snapshot ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(raw_snapshot ${catkin_LIBRARIES}
                                   ${multisense_lib_LIBRARIES})

set_target_properties(raw_snapshot
  PROPERTIES COMPILE_FLAGS "-I${PROJECT_SOURCE_DIR}/../multisense_lib/sensor_api/source/LibMultiSense")


## Color Laser Point Cloud


add_executable(color_laser_publisher src/color_laser.cpp src/point_cloud_utilities.cpp)
target_link_libraries(color_laser_publisher ${catkin_LIBRARIES})

## Install
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ros_driver raw_snapshot color_laser_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install (DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

