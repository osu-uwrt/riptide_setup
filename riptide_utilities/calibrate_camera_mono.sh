#!/bin/bash

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.041 image:=/downward/image_raw
wait
mv ~/.ros/camera_info ~/osu-uwrt/riptide_software/riptide_hardware/cfg/
