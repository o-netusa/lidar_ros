add_library(lidar_ros_driver_lib LidarRosDriver.cpp)

target_compile_features(lidar_ros_driver_lib PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros_driver_lib ${catkin_LIBRARIES} LidarDevice)

#add_dependencies(lidar_ros_driver_lib lidar_ros_driver_generate_messages_cpp)


add_executable(lidar_ros_driver LidarRosNode.cpp)
target_compile_features(lidar_ros_driver PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros_driver ${catkin_LIBRARIES} LidarDevice lidar_ros_driver_lib)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

