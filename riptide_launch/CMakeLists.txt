cmake_minimum_required(VERSION 2.8.3)
project(riptide_launch)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
