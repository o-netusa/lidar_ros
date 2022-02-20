include(CTest)
include(GoogleTest)

find_package(GTest REQUIRED)

catkin_add_gtest(lidar_ros_test DeviceTests.cpp)
target_include_directories(lidar_ros_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(lidar_ros_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} lidar_ros)
