#!/bin/bash

# Add appropriate lines to the bashrc if they do not exist
if ! grep -q "$s1" ~/.bashrc || ! grep -q "$s2" ~/.bashrc || ! grep -q "$s3" ~/.bashrc; then
    sed -i "/setup.bash/d" ~/.bashrc
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    echo "source ~/osu-uwrt/development/dependencies/install/setup.bash" >> ~/.bashrc
    echo "source ~/osu-uwrt/development/software/install/setup.bash" >> ~/.bashrc
fi