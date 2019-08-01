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
  auv_gnc
  auv_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp
)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(trans_ekf_combinator src/trans_ekf_combinator_node.cpp src/trans_ekf_combinator.cpp)
target_link_libraries(trans_ekf_combinator ${catkin_LIBRARIES})
add_dependencies(trans_ekf_combinator ${catkin_EXPORTED_TARGETS})

add_executable(gnc_thrust_converter src/gnc_thrust_converter_node.cpp src/gnc_thrust_converter.cpp)
target_link_libraries(gnc_thrust_converter ${catkin_LIBRARIES})
add_dependencies(gnc_thrust_converter ${catkin_EXPORTED_TARGETS})

