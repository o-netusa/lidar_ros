include(CTest)
include(GoogleTest)
#
find_package(rostest REQUIRED)
catkin_add_gtest(lidar_ros_test DeviceTests.cpp)
target_link_libraries(lidar_ros_test ${catkin_LIBRARIES} gtest LidarDevice)
#${PROJECT_NAME}
