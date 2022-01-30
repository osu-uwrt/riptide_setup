#!/bin/bash

# Default Linux installation doesn't have pip. Install and update for both Python versions for good measure
sudo apt-get install python3-pip --yes
python3 -m pip install --upgrade pip
sudo python3 -m pip install vcstool

if [ ! -d "../riptide_software/src" ]; then
    if type lsb_release >/dev/null 2>&1; then
        VER=$(lsb_release -sr)
    else
        echo "Linux distro not recognized"
        exit
    fi
    
    mkdir -p ../riptide_software
    mkdir -p ../riptide_software/src 

    vcs import < riptide_base.repos ../riptide_software/src

    #Assigning default value as 1: Desktop full install
    echo
    echo
    read -p "Would you like to install the simulator (Y/n)? " -n 1 answer 
    echo
    echo

    case "$answer" in
        y|Y|"" ) 
            git clone https://github.com/osu-uwrt/riptide_gazebo ../riptide_software/src/riptide_gazebo
            cd ../riptide_software/src/riptide_gazebo
            git checkout ros2
            cd ../../../riptide_setup
        ;;
    esac

else
    echo
    echo
    read -p "Would you like to force reinstallation of ROS (Y/n)? " -n 1 answer 
    echo
    echo

    case "$answer" in
        y|Y|"" ) 
            export REINSTALL=1
        ;;
    esac
fi

cd scripts/setup_scripts
./update_system.sh