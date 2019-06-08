#!/bin/bash
# Data will be saved to the computer the cameras are plugged into
echo "Please ensure cameras are running (cameras.launch or underwaterCamera.launch)"
#rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.041 camera:=/downward image:=/downward/image_raw
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.041 camera:=/underwater_cam image:=/underwater_cam/image_raw

