#!/bin/bash


cd ~/osu-uwrt/
mkdir dependencies
cd dependencies
mkdir src
cd src
git clone https://github.com/ros-drivers/pointgrey_camera_driver.git
git clone https://github.com/osu-uwrt/imu_3dm_gx4.git
git clone --recursive https://github.com/osu-uwrt/darknet_ros.git

cd ..
source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install

