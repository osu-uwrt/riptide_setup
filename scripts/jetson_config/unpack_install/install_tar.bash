#!/bin/bash 

if [ $# -ge 2 ]; then
    export ROS_DISTRO=$1
    ROS_TAR=$2
else
    echo "This script needs two arguments, ROS_DISTRO, and ROS_TAR"
    echo "./install_tar.bash <ROS_DISTRO> <ROS_TAR>"
    exit
fi

echo "Unpacking tar archive $ROS_TAR for install"

# extract archive
tar -xf $ROS_TAR