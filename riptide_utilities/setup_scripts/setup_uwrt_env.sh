#!/bin/sh

# Setup ~/.bashrc file
sh setup_bashrc.sh

# Add user to group 'uwrt' for sensor permissions
~/osu-uwrt/riptide_software/src/riptide_hardware/scripts/add_rule

# Compile Code
cd ~/osu-uwrt/riptide_software
catkin_make

echo "If no errors occurred during compilation, then everything was setup correctly"
echo "Please reboot your computer for final changes to take effect"