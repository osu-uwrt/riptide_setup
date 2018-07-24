cmake_minimum_required(VERSION 2.8.3)
project(riptide_autonomy)

find_package(catkin REQUIRED
    COMPONENTS
    riptide_msgs
    roscpp
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
