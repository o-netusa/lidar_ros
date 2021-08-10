include(CTest)
include(GoogleTest)
#
find_package(rostest REQUIRED)
catkin_add_gtest(lidar_ros_test test.cpp)
target_link_libraries(lidar_ros_test ${catkin_LIBRARIES})
