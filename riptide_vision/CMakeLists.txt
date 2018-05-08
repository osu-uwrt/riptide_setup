cmake_minimum_required(VERSION 2.8.3)
project(riptide_vision)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    riptide_msgs
    sensor_msgs
    cv_bridge
)

catkin_package()
