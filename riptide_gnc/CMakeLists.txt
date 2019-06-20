cmake_minimum_required(VERSION 2.8.3)
project(riptide_gnc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  riptide_msgs
  auv_msgs
  auv_gnc
)

find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(trans_ekf_combinator src/trans_ekf_combinator_node.cpp src/trans_ekf_combinator.cpp)
target_link_libraries(trans_ekf_combinator ${catkin_LIBRARIES})
add_dependencies(trans_ekf_combinator ${catkin_EXPORTED_TARGETS})

