#!/bin/bash

wget https://raw.githubusercontent.com/PickNikRobotics/quick-ros-install/master/ros_install.sh 
chmod 755 ros_install.sh
./ros_install.sh $ROS_DISTRO
rm ros_install.sh


