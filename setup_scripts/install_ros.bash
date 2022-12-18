#!/bin/bash

#Getting version and release number of Ubuntu
version=`lsb_release -sc`
releasenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
echo ">>> {Your Ubuntu version is: [Ubuntu $version $releasenum]}"


# add ros key to keyring
sudo apt update && sudo apt install curl gnupg lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# add ros2 repos to the sources.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
 http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
 | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


sudo apt update

echo "#######################################################################################################################"
echo ""
echo ">>>  {Starting ROS installation, this will take about 20 min. It will depends on your internet  connection}"
echo ""

read -p "Enter your install (Default is desktop):" answer 

if [ "$answer" == "base" ]; then # base install

  sudo apt install ros-${ROS_DISTRO}-ros-base -y

else # Desktop install 

  sudo apt install ros-${ROS_DISTRO}-desktop -y

fi

# install ros2 tools
sudo apt install python3-rosdep python3-colcon-common-extensions python3-colcon-clean -y 

sudo rosdep init
rosdep update
