#!/bin/bash

# make all the directories
mkdir -p ~/osu-uwrt/development
cd ~/osu-uwrt/development

# TODO query for new sim setup here

echo "Importing repositories"
vcs import < ~/osu-uwrt/riptide_setup_new/setup_scripts/dev_install/riptide.repos . --recursive

# install child dependencies and build dependencies
echo "Building dependencies"
cd ~/osu-uwrt/development/dependencies

# test for subfolders named zed and disable if nvidia-smi doesnt run
# this can also work for other packages and platforms
if ! [ -x "$(command -v nvidia-smi)" ]; then
    echo "Nvidia driver not foun. Disabling zed packages"
    python3 ~/osu-uwrt/riptide_setup_new/setup_scripts/dev_install/package_disable.py zed
fi

rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build

# add pico compilier tools
echo "Downloading Pico utils"
sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential

# build development software (this should pass)
echo "Building riptide software"
cd ~/osu-uwrt/development/software
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/osu-uwrt/development/dependencies/install/setup.bash
colcon build