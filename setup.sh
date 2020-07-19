#!/bin/bash

sudo apt install python-pip
sudo pip install vcstool

if [ ! -d "../riptide_software/src" ]; then
    mkdir -p ../riptide_software
    mkdir -p ../riptide_software/src 

    vcs import < riptide_base.repos ../riptide_software/src
fi

cd scripts/setup_scripts
./update_system.sh