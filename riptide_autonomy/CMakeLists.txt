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
  sensor_msgs
  cv_bridge
)


roslint_cpp()

set(AUTONOMY_OBJECTS
  src/tslam.cpp
  src/task_casino_gate.cpp
  src/task_path_marker.cpp
  src/task_dice_hop.cpp
  src/task_dice.cpp
  src/task_slots.cpp
  src/task_roulette.cpp
  src/task_gold_chip.cpp
  src/task_cash_in.cpp
  src/object_describer.cpp
  src/validators.cpp
)

catkin_package(INCLUDE_DIRS include)
find_package(OpenCV REQUIRED) # To include, OpenCV, this MUST go BEFORE include_directories()
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)

# if(NOT _host_name MATCHES "jetson")
#  add_executable(be_autonomous src/be_autonomous.cpp ${AUTONOMY_OBJECTS})
#  target_link_libraries(be_autonomous ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
#  add_dependencies(be_autonomous ${catkin_EXPORTED_TARGETS})
# endif()
