cmake_minimum_required(VERSION 2.8.3)
project(riptide_navigation)

find_package(catkin REQUIRED COMPONENTS control_toolbox)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
