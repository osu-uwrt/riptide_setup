#!/bin/bash

# Update ~/.bashrc File

s1="source /opt/ros/$ROS_DISTRO/setup.bash"
s2="source ~/osu-uwrt/dependencies/install/setup.bash"
s3="source ~/osu-uwrt/riptide_software/devel/setup.bash"


# Add appropriate lines to the bashrc if they do not exist
if ! grep -q "$s1" ~/.bashrc || ! grep -q "$s2" ~/.bashrc || ! grep -q "$s3" ~/.bashrc; then
    sed -i "/setup.bash/d" ~/.bashrc
    echo $s1 >> ~/.bashrc
    echo $s2 >> ~/.bashrc
    echo $s3 >> ~/.bashrc
fi