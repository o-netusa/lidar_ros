cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# MODULE_NAME is useful for logger naming
add_compile_definitions(MODULE_NAME="APPS")

find_package(Threads REQUIRED)

add_executable(lidar_ros_server lidar_ros_server.cpp)
target_compile_features(lidar_ros_server PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros_server lidar_ros cxxopts Threads::Threads)

