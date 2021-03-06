cmake_minimum_required(VERSION 3.10.2)
project(dragoon_simulation)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  rostest
  rosunit
  actionlib_msgs
  urdf
  dragoon_messages
  gazebo_ros
  pcl_ros
  pcl_conversions
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  actionlib_msgs
  dragoon_messages
  gazebo_ros
  pcl_ros
  pcl_conversions
)
include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  include
  )
# Load catkin and all dependencies required for this package

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# add gazebo plugin as library for the node 
add_library(${PROJECT_NAME} src/gazebo_api/gazebo_api.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(marker_node src/marker_draw/markers.cpp)
target_link_libraries(marker_node ${catkin_LIBRARIES})

add_executable(grid_generator src/grid_generator/grid_generator.cpp)
target_link_libraries(grid_generator ${catkin_LIBRARIES})
target_compile_options(grid_generator PRIVATE -Ofast)

add_executable(map_frame_broadcaster src/grid_generator/map_frame_broadcaster.cpp)
target_link_libraries(map_frame_broadcaster ${catkin_LIBRARIES})

# if(CATKIN_ENABLE_TESTING)

#   find_package(rostest REQUIRED)

#   # add_rostest_gtest(myrostest test/rostest.launch test/rostest.cpp)
#   # target_link_libraries(myrostest 
#   # ${catkin_LIBRARIES} 
#   # ${GTEST_LIBRARIES}
#   # )

#   catkin_add_gtest(unittest test/unittest.cpp)
#   target_link_libraries(unittest 
#   myAlgo 
#   # ${YAML_CPP_LIBRARIES}
#   ${catkin_LIBRARIES} 
#   ${GTEST_LIBRARIES}
#   )
# endif()

# target_link_libraries(simple_node
#   myAlgo
#   ${catkin_LIBRARIES}
#   )