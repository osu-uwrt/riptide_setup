#!/bin/bash

# download the bridge into its own workspace
mkdir -p ~/osu-uwrt/ros1_bridge_ws/src

pushd ~/osu-uwrt/ros1_bridge_ws > /dev/null

    pushd ./src  > /dev/null
    
        git clone -b ${ROS2_DISTRO} https://github.com/ros2/ros1_bridge.git 

    popd > /dev/null

    # source the ros1 installation
    source /opt/ros/${ROS_DISTRO}/setup.bash

    #source the ros2 installation
    source /opt/ros/${ROS2_DISTRO}/setup.bash

    # pull in the ros1 overlay
    source ~/osu-uwrt/dependencies/install/setup.bash
    source ~/osu-uwrt/riptide_software/devel/setup.bash

    # pull in the ros2 overlay
    source ~/osu-uwrt/riptide_software_ros2/install/setup.bash

    # build the bridge
    colcon build --symlink-install --cmake-force-configure

popd > /dev/null



