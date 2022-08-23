#!/bin/bash
echo "#######################################################################################################################"
echo ""
echo ">>> {Starting ROS Galactic Installation}"
echo ""
echo ">>> {Checking your Ubuntu version} "
echo ""
#Getting version and release number of Ubuntu
version=`lsb_release -sc`
releasenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
echo ">>> {Your Ubuntu version is: [Ubuntu $version $releasenum]}"
#Checking version is focal, if yes proceed othervice quit
case $version in
  "jammy" )
  ;;
  *)
    echo ">>> {ERROR: This script will only work on Ubuntu Jammy (22.04).}"
    exit 0
esac

# add ros key to keyring
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 2: Set up your keys}"
echo ""
echo ">>> {Installing curl, gnupg and lsb-release for adding keys}"
sudo apt update && sudo apt install curl gnupg lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# add ros2 repos to the sources.list
echo ""
echo ">>> {Done: Added Ubuntu repositories}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 3: Setup your sources.list}"
echo ""
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo ">>> {Done: Added Keys}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 4: Updating Ubuntu package index, this will take few seconds depending on your network connection}"
echo ""
sudo apt update

echo "#######################################################################################################################"
echo ""
echo ">>>  {Starting ROS installation, this will take about 20 min. It will depend on your internet  connection}"
echo ""

read -p "Enter your install (Default is desktop):" answer 

if [ "$answer" == "base" ]; then

  sudo apt install ros-humble-ros-base -y

else # Base install

  sudo apt install ros-humble-desktop -y

fi

# install ros2 tools
sudo apt install python3-rosdep python3-colcon-common-extensions -y 

sudo rosdep init
rosdep update

echo ""
echo ""
echo "#######################################################################################################################"
