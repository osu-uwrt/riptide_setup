#!/bin/bash

cd ~/osu-uwrt

# the only thing this script does is a recursive pull of the release repo...
# not much amazing stuff in here.. sorry (not sorry)
if [[ ! -d ~/osu-uwrt/release ]]; then
    git clone --recursive https://github.com/osu-uwrt/riptide_release.git release
else
    echo "Repository already cloned. Skipping clone..."
fi

cd ~/osu-uwrt/release

git pull

sudo apt install -y python3-scantree
pip install fabric


cd ~/osu-uwrt/riptide_setup

colcon build

# Add appropriate lines to the bashrc if they do not exist
s2="source ~/osu-uwrt/riptide_setup/install/setup.bash"
if ! grep -q "$s2" ~/.bashrc ; then
    echo "$s2" >> ~/.bashrc
fi

echo "Release repository setup"