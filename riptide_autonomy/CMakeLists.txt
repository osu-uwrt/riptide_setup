cmake_minimum_required(VERSION 2.8.3)
project(riptide_autonomy)

find_package(catkin REQUIRED COMPONENTS smach_ros roscpp actionlib actionlib_msgs riptide_msgs)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(CATKIN_DEPENDS riptide_msgs)

include_directories(${catkin_INCLUDE_DIRS})


