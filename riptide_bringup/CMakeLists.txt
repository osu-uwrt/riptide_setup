cmake_minimum_required(VERSION 2.8.3)
project(riptide_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch roscpp riptide_msgs)

catkin_package()

add_executable(thruster_validation src/thruster_validation.cpp)
add_dependencies(thruster_validation riptide_msgs_gencpp)
target_link_libraries(thruster_validation ${catkin_LIBRARIES})
include_directories(${catkin_INCLUDE_DIRS})
