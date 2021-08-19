include(CTest)
include(GoogleTest)

find_package(GTest REQUIRED)

catkin_add_gtest(lidar_ros_driver_test DeviceTests.cpp)
target_link_libraries(lidar_ros_driver_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} lidar_ros_driver_lib)
