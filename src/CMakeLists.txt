cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(lidar_ros_driver CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CXX_STD cxx_std_17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

include(DownloadProject)

set(CPPBASE_BUILD_GTEST OFF)
download_project(
  PROJ           cppbase
  GIT_REPOSITORY ssh://git@192.168.10.1:2222/lidar/cppbase
  GIT_TAG        master
)
add_subdirectory(${cppbase_SOURCE_DIR} ${cppbase_BINARY_DIR})
set(CPPBASE_INCLUDE_DIR ${cppbase_SOURCE_DIR} CACHE STRING "")

set(USE_OPEN3D OFF)
download_project(
  PROJ           lidardevice
  GIT_REPOSITORY ssh://git@192.168.10.1:2222/lidar/lidardevice
  GIT_TAG        master
)
add_subdirectory(${lidardevice_SOURCE_DIR} ${lidardevice_BINARY_DIR})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
#  DEPENDS
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CPPBASE_INCLUDE_DIR}
  ${lidardevice_INCLUDE_DIR}
)

add_subdirectory(lidar_ros_driver)
add_subdirectory(test_node)
add_subdirectory(tests)

include(ClangFormat)
clang_format_recurse("lidar_ros_driver" "test_node" "tests")


#############
## Install ##
#############

#############
## Testing ##
#############


