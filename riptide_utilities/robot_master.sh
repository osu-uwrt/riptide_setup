#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/ros/osu-uwrt/riptide_software/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.232:11311

exec "$@"