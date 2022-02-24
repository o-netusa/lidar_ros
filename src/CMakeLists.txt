cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(lidar_ros_driver VERSION 1.0 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CXX_STD cxx_std_17)

add_subdirectory(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  common_msgs
  rosbag
)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

include(DownloadProject)
add_subdirectory(external)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime common_msgs rosbag
#  DEPENDS
)

set(PARAMETER_FLAG_INCLUDE "${CMAKE_SOURCE_DIR}/lidar_ros_driver")

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CPPBASE_INCLUDE_DIR}
  ${PARAMETER_FLAG_INCLUDE}
)

add_subdirectory(lidar_ros_driver)
add_subdirectory(test_node)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif(CATKIN_ENABLE_TESTING)

#############
## Install ##
#############
include(InstallRequiredSystemLibraries)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR "${lidar_ros_driver_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${lidar_ros_driver_VERSION_MINOR}")
set(CPACK_PACKAGE_VENDOR "ONET")

include(ClangFormat)
clang_format_recurse("lidar_ros_driver" "test_node" "tests")
