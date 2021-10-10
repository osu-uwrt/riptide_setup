#!/bin/bash

# invoke for ros1 ws
cd ~/osu-uwrt/riptide_software
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r

#invoke for ros2 ws
if[ -z $ROS2_DISTRO ]; then 
    cd ~/osu-uwrt/riptide_software_ros2
    rosdep install --from-paths src --ignore-src --rosdistro $ROS2_DISTRO -y -r
fi