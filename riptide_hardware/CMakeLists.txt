cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)

find_package(catkin REQUIRED
    COMPONENTS
    roslint
    roscpp
    riptide_msgs
    pointgrey_camera_driver
)

catkin_package()
