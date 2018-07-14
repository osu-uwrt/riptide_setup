cmake_minimum_required(VERSION 2.8.3)
project(riptide_autonomy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  riptide_msgs
  geometry_msgs
)

roslint_cpp()

catkin_package(INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(tslam src/tslam.cpp)
target_link_libraries(tslam ${catkin_LIBRARIES})
add_dependencies(tslam ${catkin_EXPORTED_TARGETS})

add_executable(gate src/task.gate.cpp)
target_link_libraries(gate ${catkin_LIBRARIES})
add_dependencies(gate ${catkin_EXPORTED_TARGETS})