# Setup UWRT Environment
~/osu-uwrt/riptide_software/src/riptide_utilities/install_ros/install_ros_kinetic.sh
~/osu-uwrt/riptide_software/src/riptide_utilities/install_ros/install_dependencies.sh
echo "source ~/osu-uwrt/sim_software/devel/setup.bash" >> ~/.bashrc
echo "source ~/osu-uwrt/shared_software/devel/setup.bash" >> ~/.bashrc
echo "source ~/osu-uwrt/riptide_software/devel/setup.bash" >> ~/.bashrc
echo "#export ROS_MASTER_URI=http://riptide:11311" >> ~/.bashrc
echo "#export ROS_MASTER_URI=http://jetosn:11311" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://$(hostname):11311" >> ~/.bashrc