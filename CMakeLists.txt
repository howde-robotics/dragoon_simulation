cmake_minimum_required(VERSION 2.8.3)
project(template_ros_node)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src
)

# add executables from your source code, i.e. compile it
# .py code are not included here as they do not need to be compiled
add_executable(simple_node 
  src/simple_node/simple_node.cpp
  )
target_link_libraries(simple_node
  ${catkin_LIBRARIES}
  )
