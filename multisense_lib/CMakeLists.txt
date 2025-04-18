cmake_minimum_required(VERSION 2.8.3)
project(multisense_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge)


catkin_destinations()  # get catkin variable defines for destinations
set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(SOURCE_DIRECTORY /sensor_api)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
set (OpenCV_LIBS ${catkin_LIBRARIES})

## Mark cpp header files for installation
file(GLOB LIBMULTISENSE_HEADERS
    "sensor_api/source/Legacy/include/MultiSense/*.hh"
)

file(REMOVE_RECURSE ${BASE_DIRECTORY}/include/)
file(MAKE_DIRECTORY ${BASE_DIRECTORY}/include/${PROJECT_NAME})
file(INSTALL ${LIBMULTISENSE_HEADERS} DESTINATION ${BASE_DIRECTORY}/include/${PROJECT_NAME})
include_directories(include)


## Declare a catkin package
catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS cv_bridge
               LIBRARIES MultiSense)

set(BUILD_LEGACY_API ON CACHE BOOL "Build the new API" FORCE)

# Build the libraries
add_subdirectory(sensor_api)

# install utilities
# TODO: how to move install target of bin into devel?
install(PROGRAMS
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ChangeIpUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/DeviceInfoUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ImageCalUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ImuConfigUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ImuTestUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/LidarCalUtility
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/SaveImageUtility
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hh"
)
