#!/bin/bash

cd ~/osu-uwrt/riptide_software
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r
