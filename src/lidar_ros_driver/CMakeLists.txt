add_executable(lidar_ros_driver LidarRosDriver.cpp LidarRosNode.cpp)

target_compile_features(lidar_ros_driver PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros_driver ${catkin_LIBRARIES} LidarDevice)
#eigen spdlog_header_only stduuid asio sigslot
