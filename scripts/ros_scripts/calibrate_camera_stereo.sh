#!/bin/bash
# Data will be saved to the computer the cameras are plugged into
echo "Please ensure cameras are running (cameras.launch)"
rosrun camera_calibration cameracalibrator.py --approx 0.1 --size 8x6 --square 0.041 right:=/puddles/stereo/right/image_raw left:=/puddles/stereo/left/image_raw left_camera:=/puddles/stereo/left right_camera:=/puddles/stereo/right
echo "Check jetson's .ros/camera_info for calibration"
