cmake_minimum_required(VERSION 2.8.3)
project(cafe_robot_total)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES cafe_robot_total
  CATKIN_DEPENDS rospy
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

