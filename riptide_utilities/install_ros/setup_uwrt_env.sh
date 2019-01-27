# Setup UWRT Environment
# Install ROS and dependencies
install_ros_kinetic.sh
install_dependencies.sh

# Setup ~/.bashrc file
echo "source ~/osu-uwrt/riptide_software/devel/setup.bash" >> ~/.bashrc
echo "#export ROS_MASTER_URI=http://riptide:11311" >> ~/.bashrc
echo "#export ROS_MASTER_URI=http://jetosn:11311" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://$(hostname):11311" >> ~/.bashrc
source ~/.bashrc

# Add user to group 'uwrt' for sensor permissions
sudo ~/osu-uwrt/riptide_software/src/riptide_hardware/scripts/add_rule
echo "Please reboot your computer for final changes to take effect"
