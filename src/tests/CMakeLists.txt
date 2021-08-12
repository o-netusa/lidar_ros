include(CTest)
include(GoogleTest)
#
find_package(rostest REQUIRED COMPONENTS lidar_ros_driver)
catkin_add_gtest(lidar_ros_test DeviceTests.cpp ../lidar_ros_driver/LidarRosDriver.cpp)
target_link_libraries(lidar_ros_test ${catkin_LIBRARIES} gtest LidarDevice)
