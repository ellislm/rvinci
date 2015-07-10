cmake_minimum_required(VERSION 2.8.3)
project(rvinci_input_msg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   rvinci_input.msg
   Gripper.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
