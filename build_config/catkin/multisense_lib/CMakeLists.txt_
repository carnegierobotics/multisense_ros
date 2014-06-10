cmake_minimum_required(VERSION 2.8.3)
project(multisense_lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_destinations()  # get catkin variable defines for destinations
set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(SOURCE_DIRECTORY /sensor_api)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

# Copy our .hh files to the devel include directory (later install to target).
execute_process(COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
execute_process(COMMAND cp ${BASE_DIRECTORY}/sensor_api/source/LibMultiSense/MultiSenseChannel.hh
                           ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
execute_process(COMMAND cp ${BASE_DIRECTORY}/sensor_api/source/LibMultiSense/MultiSenseTypes.hh
                           ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include)

## Declare a catkin package
catkin_package(LIBRARIES MultiSense)

# Build the libraries
add_subdirectory(sensor_api)

## Mark cpp header files for installation
install(DIRECTORY sensor_api/source/LibMultiSense/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)

# install utilities
# TODO: how to move install target of bin into devel?
install(PROGRAMS
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ChangeIpUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/DeviceInfoUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/FlashUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ImageCalUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ImuConfigUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ImuTestUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/LidarCalUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/MultiSenseUpdater
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/SaveImageUtility
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

