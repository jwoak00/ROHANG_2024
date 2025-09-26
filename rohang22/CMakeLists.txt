cmake_minimum_required(VERSION 3.0.2)
project(rohang22)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rohang22
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  rohang22 
  src/main.cpp 
  src/algebra.cpp 
  src/control.cpp 
  src/frame.cpp 
  src/guidance.cpp
  src/log.cpp
)

target_link_libraries(rohang22 ${catkin_LIBRARIES})
