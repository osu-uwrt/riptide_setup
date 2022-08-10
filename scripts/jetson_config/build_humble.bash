#!/bin/bash

UPDATE_BLOCK_FILE=/tmp/packages_installed_this_boot
if [[ -f "$UPDATE_BLOCK_FILE" ]]; then
    echo "$UPDATE_BLOCK_FILE exists. skipping apt and rosdep stuff"

else
    touch $UPDATE_BLOCK_FILE

    echo "Upgrading system packages"

    sudo apt update
    sudo apt upgrade -y

    echo "Installing base dependencies"

    # Setup the ROS repos
    sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    sudo apt install -y build-essential cmake git python3-colcon-common-extensions python3-colcon-mixin\
    python3-flake8 python3-flake8-docstrings python3-pip python3-pytest python3-pytest-cov python3-rosdep \
    python3-setuptools python3-vcstool wget bison flex ccache clang lld llvm libc++-dev libc++abi-dev libacl1-dev \
    libpython3-dev python3-dev python-is-python3 libboost-python-dev pv

    # addtl deps

    # update mixins
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default

    sudo rosdep init && rosdep update

fi

# set some environment variables for what we are about to build
export ROS_DISTRO="humble"
REPO_FILE_NAME="${ROS_DISTRO}_base_jetson"
VCS_FILE_PATH="${HOME}/osu-uwrt/riptide_setup/jetson_config/${REPO_FILE_NAME}.repos"
META_FILE_PATH="${HOME}/osu-uwrt/riptide_setup/jetson_config/${REPO_FILE_NAME}.meta"

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

        colcon build --symlink-install --cmake-clean-cache --cmake-force-configure --metas $META_FILE_PATH \
        --packages-up-to microxrcedds_agent
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
colcon build --symlink-install --cmake-clean-cache --cmake-force-configure --metas $META_FILE_PATH

# Create a backup archive we can use to re-create everything without a new build
TAR_DIR="${HOME}/osu-uwrt/jetson_install"
TARBALL_PATH="${HOME}/osu-uwrt/${REPO_FILE_NAME}.tar.gz"

echo "Creating backup tarball ${TARBALL_PATH}"
tar cf - ${TAR_DIR} -P | pv -s $(du -sb ${TAR_DIR} | awk '{print $1}') | gzip > $TARBALL_PATH