#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y python3-vcstool 

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update

sudo apt-get install -y ros-kinetic-desktop

rosdep init
rosdep update


mkdir  ../riptide_software/
mkdir ../riptide_software/src
cd ..

cp repos/riptide_base.repos .



vcs import < riptide_base.repos src



sudo rosdep fix-permissions

cd src
cd riptide_utilities
cd setup_scripts

./setup_uwrt_env.sh

cd ..
cd ..
cd ..
cd repos

bash SetupUWRTDocker


