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

vcs import < ~/osu-uwrt/riptide_setup/scripts/setup_scripts/dependencies.repos . --recursive

cd ..

# create and build micro_ros agent
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r
touch ~/osu-uwrt/dependencies/isaac_ros_pose_estimation/COLCON_IGNORE
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

# add pico compilier tools
sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential

if [ -d ~/osu-uwrt/riptide_software/src/riptide_gazebo ] 
then
  echo "Downloading sim dependencies..."
  vcs import < ~/osu-uwrt/riptide_setup/scripts/setup_scripts/gazebo_dependencies.repos . --recursive
  rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r
  colcon build
else
  echo "No riptide_gazebo found. Not downloading sim dependencies."
fi 
