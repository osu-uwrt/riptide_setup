cmake_minimum_required(VERSION 2.8.3)
project(riptide_serial)

find_package(catkin REQUIRED COMPONENTS)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
