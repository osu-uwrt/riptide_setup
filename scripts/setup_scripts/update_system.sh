#!/bin/bash

if [[ $UID = 0 ]] ; then
  echo "Please dont use sudo command with script"
  exit
fi

if [ -z "$ROS_DISTRO" ]; then
    if type lsb_release >/dev/null 2>&1; then
        VER=$(lsb_release -sr)
        if [ $VER == "18.04" ]; then
            ROS_DISTRO="melodic"
        elif [ $VER == "16.04" ]; then
            ROS_DISTRO="kinetic"
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

# Install ros and dependencies
./install_ros.sh
source /opt/ros/$ROS_DISTRO/setup.bash
./install_rosdeps.sh
source /opt/ros/$ROS_DISTRO/setup.bash

# Install Point grey drivers, on melodic its a custom ros package
if [ $ROS_DISTRO == "melodic" ]; then
    if [ ! -f /usr/lib/libflycapture.so.2 ]; then
        printf "\n\nPlease ensure flycaptue is installed from this repo: https://github.com/Juched/flycap-mirror\n\n\n"
        exit
    fi
else
    ./install_point_grey_drivers.sh
fi

# Install all custon ros packages
./install_custom_ros_packages.sh
source ~/osu-uwrt/dependencies/install/setup.bash

# Install Ceres and Eigen
sudo ./install_ceres.sh
sudo ./install_eigen.sh

# Setup ~/.bashrc and vscode
./setup_bashrc.sh
./setup_vscode.sh
sudo ./setup_hosts.sh

# Add user to group 'uwrt' for sensor permissions
sudo ~/osu-uwrt/riptide_software/src/riptide_hardware/scripts/add_rule

# Compile Code
cd ~/osu-uwrt/riptide_software
catkin clean -y
catkin build

echo "If no errors occurred during compilation, then everything was setup correctly"
echo "Please reboot your computer for final changes to take effect"