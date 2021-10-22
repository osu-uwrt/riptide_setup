#!/bin/bash

# create the ros2 packages dir
mkdir -p ../riptide_software_ros2
mkdir -p ../riptide_software_ros2/src 

# pull in all of the ros2 dependencies
vcs import < riptide_ros2.repos ../riptide_software_ros2/src
source /opt/ros/${ROS2_DISTRO}/setup.bash

# build what we just pulled in to make sure it works
cd ../riptide_software_ros2
colcon build