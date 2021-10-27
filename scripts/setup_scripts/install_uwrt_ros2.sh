#!/bin/bash

cd ~/osu-uwrt

# create the ros2 packages dir
mkdir -p ./riptide_software_ros2/src 
pushd ./riptide_software_ros2 > /dev/null
    pushd ./src > /dev/null

        # pull in all of the ros2 dependencies
        vcs import < ~/osu-uwrt/riptide_setup/riptide_ros2.repos .
        source /opt/ros/${ROS2_DISTRO}/setup.bash

    popd > /dev/null


    # build what we just pulled in to make sure it works
    source /opt/ros/$ROS2_DISTRO/setup.bash

    # create and build agent
    colcon build
    source install/local_setup.bash
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash

    # add pico compilier tools
    sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential

    # generate fw dir
    mkdir -p ../riptide_software_ros2/firmware 

    # recursive clone of titan fw
    pushd ./firmware > /dev/null

        git clone --recursive https://github.com/osu-uwrt/titan_firmware.git

    popd > /dev/null

popd > /dev/null
