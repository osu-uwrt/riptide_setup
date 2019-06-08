#!/bin/bash


cd ~/osu-uwrt/
mkdir dependencies
cd dependencies
mkdir src
cd src
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
apt-get update
apt-get install python-rosdep -y
'which rosdep' init
rosdep update
rosdep install --default-yes --from-paths ./ --rosdistro kinetic


git clone https://github.com/ros-drivers/pointgrey_camera_driver.git
git clone https://github.com/osu-uwrt/imu_3dm_gx4.git
git clone --recursive https://github.com/osu-uwrt/darknet_ros.git

cd ..

rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r

source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install

