#!/bin/bash


cd ~/osu-uwrt/
mkdir -p dependencies
cd dependencies

git -C MYNT pull || git clone https://github.com/slightech/MYNT-EYE-D-SDK.git MYNT
cd MYNT
make init
make ros
cd ..

mkdir src
cd src

vcs import < ~/osu-uwrt/riptide_setup/scripts/setup_scripts/dependencies.repos . --recursive

if [ -d ~/osu-uwrt/riptide_software/src/riptide_gazebo ] 
then
  echo "Downloading sim dependencies..."
  git -C uuv_simulator pull || git clone https://github.com/osu-uwrt/uuv_simulator.git
else
  echo "No riptide_gazebo found. Not downloading sim dependencies."
fi 

cd ..

rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r

catkin config --install --extend ~/osu-uwrt/dependencies/MYNT/wrappers/ros/devel --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
