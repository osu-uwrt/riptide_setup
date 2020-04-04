#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y python3-vcstool 

mkdir -p ../riptide_software
mkdir -p ../riptide_software/src 

vcs import < riptide_base.repos ../riptide_software/src

cd scripts/setup_scripts
./update_system.sh