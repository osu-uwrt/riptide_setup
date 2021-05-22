#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/osu-uwrt/riptide_software/devel/setup.bash
export ROS_MASTER_URI=http://venus:11311

exec "$@"
