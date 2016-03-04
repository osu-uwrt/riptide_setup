cmake_minimum_required(VERSION 2.8.3)
project(riptide_teleop)

find_package(catkin REQUIRED COMPONENTS joy roslaunch)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
