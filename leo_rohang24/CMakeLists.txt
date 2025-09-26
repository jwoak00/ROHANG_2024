cmake_minimum_required(VERSION 3.0.2)
project(leo_rohang24)

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
  LIBRARIES leo_rohang24
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  leo_rohang24 
  src/main.cpp 
  src/algebra.cpp 
  src/control.cpp 
  src/frame.cpp 
  src/guidance.cpp
  src/log.cpp
)

target_link_libraries(leo_rohang24 ${catkin_LIBRARIES})
