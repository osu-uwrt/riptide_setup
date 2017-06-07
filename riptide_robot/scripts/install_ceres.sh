#!/bin/bash

mkdir -p ~/osu-uwrt/ceres_ws/src && cd ~/osu-uwrt/ceres_ws/src

wget -qO- http://ceres-solver.org/ceres-solver-1.12.0.tar.gz | tar zxf -

rosdep install --from-path .

cd .. && catkin_make_isolated
