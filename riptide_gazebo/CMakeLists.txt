cmake_minimum_required(VERSION 2.8.3)
project(riptide_gazebo)

find_package(catkin REQUIRED COMPONENTS)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_riptide_thrust
)

add_library(
  gazebo_ros_riptide_thrust
  src/gazebo_ros_riptide_thrust.cpp
)

target_link_libraries(
  gazebo_ros_riptide_thrust
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)
