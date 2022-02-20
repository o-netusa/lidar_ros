add_library(lidar_ros LidarRosDriver.cpp)
target_compile_features(lidar_ros PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros PUBLIC ${catkin_LIBRARIES} LidarDevice)

add_executable(lidar_ros_driver LidarRosNode.cpp)
target_compile_features(lidar_ros_driver PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros_driver PUBLIC ${catkin_LIBRARIES} LidarDevice lidar_ros)

install(TARGETS lidar_ros DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install config and launch files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../config DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/lidar_ros_driver)
install(FILES ${CMAKE_SOURCE_DIR}/../launch/run.launch DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/../launch)

# install license and readme
install(FILES ${CMAKE_SOURCE_DIR}/../LICENSE.txt DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/../)
install(FILES ${CMAKE_SOURCE_DIR}/../README.md DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/../)
