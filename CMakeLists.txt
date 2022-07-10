cmake_minimum_required(VERSION 3.0.2)
project(edge_brush)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  pcl_conversions
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_message_files(
  FILES
  edge.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARIES_DIRS}
)

add_executable(edgeDetection src/edgeDetection_node.cpp src/edgeDetection.cpp)
target_link_libraries(edgeDetection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pidControl src/pidControl.cpp)
target_link_libraries(pidControl ${catkin_LIBRARIES})