cmake_minimum_required(VERSION 2.8.3)
project(multisense_lib)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(SOURCE_DIRECTORY /sensor_api)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/lib)

# Move our .hh files to the include directory
execute_process(COMMAND mkdir -p ${BASE_DIRECTORY}/include/multisense_lib)
execute_process(COMMAND cp ${BASE_DIRECTORY}/sensor_api/source/LibMultiSense/MultiSenseChannel.hh ${BASE_DIRECTORY}/include/multisense_lib)
execute_process(COMMAND cp ${BASE_DIRECTORY}/sensor_api/source/LibMultiSense/MultiSenseTypes.hh ${BASE_DIRECTORY}/include/multisense_lib)



## Declare a catkin package
catkin_package(INCLUDE_DIRS include
               LIBRARIES MultiSense)

# Build the libraries
add_subdirectory(sensor_api)

