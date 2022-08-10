#!/bin/bash

FILE=/tmp/package_installed_this_boot
if [[ -f "$FILE" ]]; then
    echo "$FILE exists. skipping apt and rosdep stuff"

else
    touch $FILE

    echo "Upgrading system packages"

    sudo apt update
    sudo apt upgrade -y

    echo "Installing base dependencies"

    # Setup the ROS repos
    sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


    sudo apt install -y build-essential cmake git python3-colcon-common-extensions python3-colcon-mixin\
    python3-flake8 python3-flake8-docstrings python3-pip python3-pytest python3-pytest-cov python3-rosdep \
    python3-setuptools python3-vcstool wget bison flex ccache clang lld llvm libc++-dev libc++abi-dev libacl1-dev \
    libpython3-dev python3-dev python-is-python3 libboost-python-dev

    # addtl deps

    # update mixins
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default

    sudo rosdep init && rosdep update

fi

# make a local directory to pull sources into
echo "Creating local source checkout"


mkdir -p ~/osu-uwrt/jetson_install/src
cd ~/osu-uwrt/jetson_install/

# pull down the needed repos
vcs import ./src < ~/osu-uwrt/riptide_setup/jetson_config/humble_base_jetson.repos

export ROS_DISTRO="humble"

rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

echo "Executing ROS build"

# meta file performs the following patches as of 8/7/22
# for python_orocos_kdl_vendor: missing python3 https://www.reddit.com/r/cmake/comments/otxfb4/comment/h724us5/  feck you from a salty engineer <3
# for rclpy: missing python3 and issues with pybind11 https://github.com/ros2/rclpy/issues/920
# for tf2_py, tf2_geometry_msgs, rosbag2_py missing python3 

# this build forces a full clean configure and rebuild of all packages. this should make everything relatively compliant assuming
# the current repos remain buildable (failurehttps://github.com/micro-ROS/micro-ROS-Agent.gits do make it into release sometimes...)
colcon build --symlink-install --cmake-clean-cache --cmake-force-configure --metas ~/osu-uwrt/riptide_setup/jetson_config/colcon.meta

