#!/bin/bash

mkdir -p ~/osu-uwrt/imu_3dm_gx4
cd ~/osu-uwrt/imu_3dm_gx4

git clone https://github.com/osu-uwrt/imu_3dm_gx4.git src
sudo ~/osu-uwrt/imu_3dm_gx4/src/script/add_rule

sudo chown -R $USER /opt/ros
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install

rm -rf ~/osu-uwrt/imu_3dm_gx4
