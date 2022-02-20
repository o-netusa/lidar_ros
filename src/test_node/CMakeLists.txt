add_executable(test_node test_node.cpp)

target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node common_msgs_generate_messages_cpp)
