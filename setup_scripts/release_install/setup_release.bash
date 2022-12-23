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

echo "Release repository setup"