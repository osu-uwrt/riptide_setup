cmake_minimum_required(VERSION 2.8.3)
project(riptide_autonomy)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

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

set(${PROJECT_NAME}_task_objects
  src/tslam.cpp
)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(be_autonomous src/be_autonomous.cpp ${${PROJECT_NAME}_task_objects})
target_link_libraries(be_autonomous ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(be_autonomous ${catkin_EXPORTED_TARGETS})
