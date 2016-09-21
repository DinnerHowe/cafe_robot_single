cmake_minimum_required(VERSION 2.8.3)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  nav_msgs
)


## Generate messages in the 'msg' folder
 #add_message_files(
 #  FILES
 #)

 generate_messages(
   DEPENDENCIES
   std_msgs nav_msgs
 )

catkin_package(
  CATKIN_DEPENDS rospy message_runtime std_msgs nav_msgs
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

