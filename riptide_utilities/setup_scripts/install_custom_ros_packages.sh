#!/bin/bash


cd ~/osu-uwrt/
mkdir dependencies
cd dependencies
mkdir src
cd src
wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get update
apt-get install python-rosdep -y

rosdep update
rosdep install --default-yes --from-paths ./ --rosdistro kinetic
rosdep install pointgrey_camera_driver

git clone https://github.com/ros-drivers/pointgrey_camera_driver.git
git clone https://github.com/osu-uwrt/imu_3dm_gx4.git
git clone --recursive https://github.com/osu-uwrt/darknet_ros.git

git clone https://github.com/osu-uwrt/nortek_dvl.git

cd ..

rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r

source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install
