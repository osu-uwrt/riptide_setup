cmake_minimum_required(VERSION 2.8.3)
project(riptide_gnc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

find_package(Eigen3 3.3 REQUIRED)

catkin_package(INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

# src/kalman_filter.cpp
# src/linear_motion_ekf_suite.cpp
# src/pose_ekf.cpp

#set(POSE_EDKF
#  src/auv_math_lib.cpp
#)

#add_executable(pose_edkf_interface src/pose_edkf_interface.cpp ${POSE_EDKF})
#target_link_libraries(pose_edkf_interface ${catkin_LIBRARIES})
#add_dependencies(pose_edkf_interface ${catkin_EXPORTED_TARGETS})

add_library(gnc src/kalman_filter.cpp)
