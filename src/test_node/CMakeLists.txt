add_executable(test_node test_node.cpp)

target_include_directories(test_node PRIVATE ${CMAKE_SOURCE_DIR}/lidar_ros_driver)
target_link_libraries(test_node PUBLIC ${catkin_LIBRARIES})
add_dependencies(test_node common_msgs_generate_messages_cpp)
