#!/bin/bash

if [[ $UID != 0 ]]; then
	echo "Please run install_rosdeps.sh as root"
	exit 1
fi

cd ~/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/packages/src
git clone https://github.com/osu-uwrt/imu_3dm_gx4.git
git clone https://github.com/osu-uwrt/darknet_ros.git
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install

