#!/bin/bash

# set some environment variables for what we are about to build
export ROS_DISTRO="humble"
REPO_FILE_NAME="${ROS_DISTRO}_base_jetson"
VCS_FILE_PATH="${HOME}/osu-uwrt/riptide_setup/scripts/jetson_config/meta_files/${REPO_FILE_NAME}.repos"
META_FILE_PATH="${HOME}/osu-uwrt/riptide_setup/scripts/jetson_config/meta_files/${REPO_FILE_NAME}.meta"

mkdir -p ~/osu-uwrt/jetson_install/src
cd ~/osu-uwrt/jetson_install/

# Hash the vcs file to find changes on subsequent builds
ARCHIVED_SUM_FILE=/tmp/vcs_checkout_sum.md5
if [[ ! -f $ARCHIVED_SUM_FILE ]]; then
    echo "checksum not found, creating"
    md5sum $VCS_FILE_PATH > ${ARCHIVED_SUM_FILE}
    FIRST_RUN=1
fi
if md5sum -c ${ARCHIVED_SUM_FILE} && [ -z ${FIRST_RUN+x} ]; then
    echo "Checkout unmodified"
else
    #overwrite the old 
    md5sum $VCS_FILE_PATH > ${ARCHIVED_SUM_FILE}

    echo "Updating sources"

    # pull down the needed repos
    vcs import ./src < $VCS_FILE_PATH

    # make sure all rosdeps are installed
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

fi


# need to build XRCE agent first to allow offline fully replicable builds
# see https://github.com/micro-ROS/micro-ROS-Agent/issues/161 
if grep -q micro_ros_agent "$VCS_FILE_PATH"; then
    # make sure the xrce agent is in the 
    if colcon list | grep -q microxrcedds_agent; then
        echo "Detected build of Micro ROS Agent. Building XRCE Agent first!"

        colcon build --merge-install --cmake-clean-cache --cmake-force-configure --metas $META_FILE_PATH \
        --packages-up-to microxrcedds_agent

        source ./install/setup.bash
    else
        echo "Failed to detect XRCE agent when asked to build Micro ROS Agent, build will now halt"
        exit
    fi
fi

echo "Executing ROS build"

# meta file performs the following patches as of 8/7/22
# for python_orocos_kdl_vendor: missing python3 https://www.reddit.com/r/cmake/comments/otxfb4/comment/h724us5/  feck you from a salty engineer <3
# for rclpy: missing python3 and issues with pybind11 https://github.com/ros2/rclpy/issues/920
# for tf2_py, tf2_geometry_msgs, rosbag2_py missing python3 

# this build forces a full clean configure and rebuild of all packages. this should make everything relatively compliant assuming
# the current repos remain buildable (failurehttps://github.com/micro-ROS/micro-ROS-Agent.gits do make it into release sometimes...)
colcon build --merge-install --cmake-clean-cache --cmake-force-configure --metas $META_FILE_PATH

# Create a backup archive we can use to re-create everything without a new build
cd ${HOME}/osu-uwrt/
TAR_DIR="./jetson_install"
TARBALL_PATH="./${REPO_FILE_NAME}.tar.gz"

echo "Creating backup tarball ${TARBALL_PATH}"
tar cf - ${TAR_DIR} -P | pv -s $(du -sb ${TAR_DIR} | awk '{print $1}') | gzip > $TARBALL_PATH