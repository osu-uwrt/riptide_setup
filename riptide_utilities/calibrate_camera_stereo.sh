#!/bin/bash

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.041 right:=/stereo/right/image_raw left:=/stereo/left/image_raw left_camera:=/stereo/left right_camera:=/stereo/right
wait
mv ~/.ros/camera_info ~/osu-uwrt/riptide_software/riptide_hardware/cfg
