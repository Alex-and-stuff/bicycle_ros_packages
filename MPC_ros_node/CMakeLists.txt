cmake_minimum_required(VERSION 2.8.3)
PROJECT (MPC_ros_node)
FIND_PACKAGE(CUDA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

SET(CUDA_NVCC_FLAGS "-arch=sm_35" CACHE STRING "nvcc flags" FORCE)
SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC) 
CUDA_ADD_LIBRARY(TestLib ${LIB_TYPE} src/kernel.cu)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

ADD_EXECUTABLE(MPC_ros_node src/main.cpp)
ADD_DEPENDENCIES(MPC_ros_node TestLib)
TARGET_LINK_LIBRARIES(MPC_ros_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   TestLib
)
