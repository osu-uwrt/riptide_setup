#!/bin/bash


cd ~/osu-uwrt/
mkdir dependencies
cd dependencies
mkdir src
cd src

if [ $ROS_DISTRO == "melodic" ]; then
    git -C pointgrey_camera_driver pull || git clone https://github.com/ros-drivers/pointgrey_camera_driver.git
fi
git -C imu_3dm_gx4 pull || git clone https://github.com/osu-uwrt/imu_3dm_gx4.git
git -C darknet_ros pull --recurse-submodules || git clone --recursive https://github.com/osu-uwrt/darknet_ros.git
git -C nortek_dvl pull || git clone https://github.com/osu-uwrt/nortek_dvl.git
cd ..

rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r

catkin config --install --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
