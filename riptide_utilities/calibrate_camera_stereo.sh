#!/bin/bash
# Data will be saved to the computer the cameras are plugged into
echo "Please ensure cameras are running (cameras.launch)"
rosrun camera_calibration cameracalibrator.py --approx 0.1 --size 8x6 --square 0.041 right:=/stereo/right/image_raw left:=/stereo/left/image_raw left_camera:=/stereo/left right_camera:=/stereo/right

