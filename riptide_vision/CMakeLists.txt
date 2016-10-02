cmake_minimum_required(VERSION 2.8.3)
project(riptide_vision)

find_package(catkin REQUIRED COMPONENTS roslint camera_calibration cv_bridge pointgrey_camera_driver)

catkin_package()

roslint_python()

include_directories(${catkin_INCLUDE_DIRS})
