# Setup UWRT Environment
# Install ROS and dependencies
sh install_ros_kinetic.sh
sh install_dependencies.sh

# Setup ~/.bashrc file
sh setup_bashrc.sh

# Add user to group 'uwrt' for sensor permissions
sudo ~/osu-uwrt/riptide_software/src/riptide_hardware/scripts/add_rule

# Compile Code
cd ~/osu-uwrt/riptide_software
catkin_make

echo "Please reboot your computer for final changes to take effect"
