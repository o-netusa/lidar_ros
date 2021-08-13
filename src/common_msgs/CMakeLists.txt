cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
   FILES
   parameter_msgs.msg
 )
generate_messages(
    DEPENDENCIES
      std_msgs
    )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES common_msg
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
