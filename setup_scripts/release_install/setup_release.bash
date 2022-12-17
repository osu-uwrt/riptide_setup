#!/bin/bash

cd ~/osu-uwrt

# the only thing this script does is a recursive pull of the release repo...
# not much amazing stuff in here.. sorry (not sorry)
git clone --recursive https://github.com/osu-uwrt/riptide_release.git

echo "Release repository setup"