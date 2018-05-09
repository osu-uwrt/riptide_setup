#!/bin/bash

source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install
echo "Exiting root"
