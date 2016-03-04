cmake_minimum_required(VERSION 2.8.3)
project(riptide_autonomy)

find_package(catkin REQUIRED COMPONENTS smach_ros)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
