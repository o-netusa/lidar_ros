cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
   FILES
   ParameterMsg.msg
 )
add_service_files(
    DIRECTORY srv
    FILES
    LidarRosService.srv
    )
generate_messages(
    DEPENDENCIES
      std_msgs
    )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
