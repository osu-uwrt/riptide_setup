# Setup UWRT Environment
# Install ROS and dependencies
~/osu-uwrt/riptide_software/src/riptide_utilities/install_ros/install_ros_kinetic.sh
~/osu-uwrt/riptide_software/src/riptide_utilities/install_ros/install_dependencies.sh

# Setup ~/.bashrc file
echo "source ~/osu-uwrt/sim_software/devel/setup.bash" >> ~/.bashrc
echo "source ~/osu-uwrt/shared_software/devel/setup.bash" >> ~/.bashrc
echo "source ~/osu-uwrt/riptide_software/devel/setup.bash" >> ~/.bashrc
echo "#export ROS_MASTER_URI=http://riptide:11311" >> ~/.bashrc
echo "#export ROS_MASTER_URI=http://jetosn:11311" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://$(hostname):11311" >> ~/.bashrc
source ~/.bashrc

# Add user to group 'uwrt' for sensor permissions
sudo ~/osu-uwrt/riptide_software/src/riptide_hardware/scripts/add_rule
echo "Please reboot your computer for final changes to take effect\n"