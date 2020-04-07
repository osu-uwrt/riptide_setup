#!/bin/bash


cd ~/osu-uwrt/
mkdir dependencies
cd dependencies

git -C MYNT pull || git clone https://github.com/slightech/MYNT-EYE-D-SDK.git MYNT
cd MYNT
make init
make ros
cd ..

mkdir src
cd src

git -C imu_3dm_gx4 pull || git clone https://github.com/osu-uwrt/imu_3dm_gx4.git
git -C darknet_ros pull --recurse-submodules || git clone --recursive https://github.com/osu-uwrt/darknet_ros.git
git -C nortek_dvl pull || git clone https://github.com/osu-uwrt/nortek_dvl.git
git -C flexbe_app pull || git clone https://github.com/FlexBE/flexbe_app.git
cd ..

rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r

catkin config --install --extend ~/osu-uwrt/dependencies/MYNT/wrappers/ros/devel --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
