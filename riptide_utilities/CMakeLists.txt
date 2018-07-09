cmake_minimum_required(VERSION 2.8.3)
project(riptide_utilities)

find_package(catkin REQUIRED
    COMPONENTS
    sensor_msgs
    rospy
    riptide_msgs
)
catkin_package()
catkin_python_setup()

catkin_install_python(PROGRAMS ps3_control.py
    DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION))
include_directories(include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
