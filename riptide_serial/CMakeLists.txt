cmake_minimum_required(VERSION 2.8.3)
project(riptide_serial)

find_package(catkin REQUIRED COMPONENTS riptide_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
