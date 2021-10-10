#!/bin/bash

if [[ $UID = 0 ]] ; then
  echo "Please dont use sudo with this script"
  exit
fi

if [[ $1 != '--nodownload' ]] ; then
  git pull
  ./update_system.sh --nodownload
  exit
fi

if [ -z "$ROS_DISTRO" ]; then
    if type lsb_release >/dev/null 2>&1; then
        VER=$(lsb_release -sr)
        if [ $VER == "18.04" ]; then
            ROS_DISTRO="melodic"
        elif [ $VER == "16.04" ]; then
            ROS_DISTRO="kinetic"
        elif [ $VER == "20.04" ]; then
            ROS_DISTRO="noetic"
            ROS2_DISTRO="galactic"
        else
            echo "Linux version not recognized"
            exit
        fi
        echo "Ros distribution $ROS_DISTRO selected"
        export ROS_DISTRO
    else
        echo "Linux distro not recognized"
        exit
    fi
fi

# Install ros
if [ ! -d "/opt/ros/$ROS_DISTRO" ]; then
    if [ $ROS_DISTRO == "noetic" ]; then
        #./install_noetic.sh
        ./install_ros1_ros2.sh
    else
        echo "Ubuntu version not supported"
        exit
    fi
fi
source /opt/ros/$ROS_DISTRO/setup.bash

# Install dependencies
./install_rosdeps.sh
source /opt/ros/$ROS_DISTRO/setup.bash

# Install all custom ros packages
./install_custom_ros_packages.sh
source ~/osu-uwrt/dependencies/install/setup.bash

# Setup ~/.bashrc and vscode
./setup_bashrc.sh
./setup_vscode.sh
sudo ./setup_hosts.sh

# Add user to group 'uwrt' for sensor permissions
sudo hardware/add_rule

# Compile Code
cd ~/osu-uwrt/riptide_software
catkin clean -y
catkin build

echo "If no errors occurred during compilation, then everything was setup correctly"
echo "Please reboot your computer for final changes to take effect"