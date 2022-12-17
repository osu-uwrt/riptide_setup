#!/bin/bash

# Default Linux installation doesn't have pip. Install and update for both Python versions for good measure
sudo apt-get install python3-pip --yes
python3 -m pip install --upgrade pip
sudo python3 -m pip install vcstool


if type lsb_release >/dev/null 2>&1; then
    VER=$(lsb_release -sr)
else
    echo "Linux distro not recognized"
    exit
fi

if [ -d "~/osu-uwrt/development" ]; then
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

cd ~/osu-uwrt/riptide_setup
~/osu-uwrt/riptide_setup/scripts/update_system.sh