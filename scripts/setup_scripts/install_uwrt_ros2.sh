#!/bin/bash

cd ~/osu-uwrt

# create the ros2 packages dir
mkdir -p ./riptide_software/src 
pushd ./riptide_software > /dev/null
    pushd ./src > /dev/null

        # pull in all of the ros2 dependencies
        vcs import < ~/osu-uwrt/riptide_setup/riptide_ros2.repos .
        source /opt/ros/${ROS_DISTRO}/setup.bash

    popd > /dev/null

    # generate fw dir
    mkdir -p ../riptide_software/firmware 

    # recursive clone of titan fw
    pushd ./firmware > /dev/null

        git clone --recursive https://github.com/osu-uwrt/titan_firmware.git
        touch COLCON_IGNORE

    popd > /dev/null

    # Pull local dependencies
    source /opt/ros/$ROS_DISTRO/setup.bash
    source ~/osu-uwrt/dependencies/install/setup.bash

    # Install dependencies
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r

    colcon build

popd > /dev/null
