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

SET(CUDA_NVCC_FLAGS "-arch=sm_75" CACHE STRING "nvcc flags" FORCE)
SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC) 
CUDA_ADD_LIBRARY(Test2Lib ${LIB_TYPE} src/kernel_obs0718.cu)
#CUDA_ADD_LIBRARY(Test2Lib ${LIB_TYPE} src/kernel_obs.cu)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

ADD_EXECUTABLE(MPC_ros_node src/main_obs0718.cpp)
#ADD_EXECUTABLE(MPC_ros_node src/main_obs.cpp)
ADD_DEPENDENCIES(MPC_ros_node Test2Lib)
TARGET_LINK_LIBRARIES(MPC_ros_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   Test2Lib
)
