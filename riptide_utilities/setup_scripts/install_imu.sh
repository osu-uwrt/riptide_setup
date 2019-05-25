git clone https://github.com/osu-uwrt/imu_3dm_gx4.git
cd imu_3dm_gx4
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install
cd ..
rm -rf imu_3dm_gx4
