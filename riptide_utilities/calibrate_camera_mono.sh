#!/bin/bash
# Cameras must be running on the same machine as the calibrator
roslaunch riptide_hardware cameras.launch
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.041 camera:=/downward image:=/downward/image_raw
wait
mv ~/.ros/camera_info ~/osu-uwrt/riptide_software/riptide_hardware/cfg/
