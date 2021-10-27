#!/bin/bash

cd ~/osu-uwrt

# create the ros2 packages dir
mkdir -p ../riptide_software_ros2/src 

# generate fw dir
mkdir -p ../riptide_software_ros2/firmware 

# pull in all of the ros2 dependencies
vcs import < riptide_ros2.repos ../riptide_software_ros2/src
source /opt/ros/${ROS2_DISTRO}/setup.bash

# build what we just pulled in to make sure it works
cd ../riptide_software_ros2
colcon build

# add pico compilier tools
sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential

# recursive clone of titan fw
git clone --recursive 

# create and build agent
