#!/bin/bash

if [[ $UID == 0 ]]; then
	echo "Please do not start imu driver installation as root"
	exit 1
fi

cd ~/osu-uwrt/imu_3dm_gx4
catkin_make install
sudo su -c src/script/install_imu_driver_root.sh root

echo "IMU driver installation complete"
