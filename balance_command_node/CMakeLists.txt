cmake_minimum_required(VERSION 2.8.3)
PROJECT (balance_command_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

ADD_EXECUTABLE(balance_command_node src/main.cpp)
TARGET_LINK_LIBRARIES(balance_command_node
   ${catkin_LIBRARIES}
)
