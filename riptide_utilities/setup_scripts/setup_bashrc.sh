#!/bin/sh

# Update ~/.bashrc File

s1="source ~/osu-uwrt/riptide_software/devel/setup.bash"
s2="export ROS_MASTER_URI=http://riptide:11311"
s3="export ROS_MASTER_URI=http://jetson:11311"
s4="export ROS_MASTER_URI=http://$(hostname):11311"

# Add appropriate lines to the bashrc if they do not exist
if ! grep -q "$s1" ~/.bashrc; then
    echo $s1 ~/.bashrc
fi

if ! grep -q "$s2" ~/.bashrc; then
    echo "#"$s2 ~/.bashrc
fi

if ! grep -q "$s3" ~/.bashrc; then
    echo "#"$s3 ~/.bashrc
fi

if ! grep -q "$s4" ~/.bashrc; then
    echo $s4 ~/.bashrc
fi

source ~/.bashrc