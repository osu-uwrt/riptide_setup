#!/bin/bash


cd ~/osu-uwrt/
mkdir -p dependencies
cd dependencies

git -C MYNT pull || git clone https://github.com/slightech/MYNT-EYE-D-SDK.git MYNT
pushd ./MYNT
    make init
    # run make install to only build SDK, the wrapper happens in the next steps
    make install
popd > /dev/null

mkdir src
cd src

 # create and build micro_ros agent
colcon build
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

# add pico compilier tools
sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential

vcs import < ~/osu-uwrt/riptide_setup/scripts/setup_scripts/dependencies.repos . --recursive

if [ -d ~/osu-uwrt/riptide_software/src/riptide_gazebo ] 
then
  echo "Downloading sim dependencies..."
  vcs import < ~/osu-uwrt/riptide_setup/scripts/setup_scripts/gazebo_dependencies.repos . --recursive
else
  echo "No riptide_gazebo found. Not downloading sim dependencies."
fi 

cd ..

rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r

colcon build
