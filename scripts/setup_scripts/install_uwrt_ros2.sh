#!/bin/bash

cd ~/osu-uwrt

# create the ros2 packages dir
mkdir -p ./riptide_software/src 
pushd ./riptide_software > /dev/null
    pushd ./src > /dev/null

        # pull in all of the ros2 dependencies
        vcs import < ~/osu-uwrt/riptide_setup/riptide_ros2.repos .
        source /opt/ros/${ROS2_DISTRO}/setup.bash

    popd > /dev/null

    # build what we just pulled in to make sure it works
    source /opt/ros/$ROS2_DISTRO/setup.bash

    # generate fw dir
    mkdir -p ../riptide_software_ros2/firmware 

    # recursive clone of titan fw
    pushd ./firmware > /dev/null

        git clone --recursive https://github.com/osu-uwrt/titan_firmware.git

    popd > /dev/null

    # Install dependencies
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r

popd > /dev/null
