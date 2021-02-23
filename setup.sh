#!/bin/bash

# Default Linux installation doesn't have pip. Install and update for both Python versions for good measure
sudo apt-get install python3-pip --yes
python3 -m pip install --upgrade pip
sudo python3 -m pip install vcstool

if [ ! -d "../riptide_software/src" ]; then
    mkdir -p ../riptide_software
    mkdir -p ../riptide_software/src 

    vcs import < riptide_base.repos ../riptide_software/src
fi

cd scripts/setup_scripts
./update_system.sh