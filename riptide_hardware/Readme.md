# The riptide_hardware Package
This package handles the hardware-software interface with all of our AUVs sensors: IMU, depth sensor, cameras, and acoustics

# Package Struture
# Config Files (cfg/)
This folder contains .yaml files with parameters needed by some of our ROS nodes. The main files are calibration paramters to undistort the images from our PointGrey Blackfly USB 3.0 cameras, as well as geospatial parameters for the LordMicrostrain IMU 3DM-GX4.

# Source Code (src/)
## Active ROS Nodes
### imu_processor
### depth_processor
### undistort_camera
### acoustics
## Utility ROS Nodes
### calibrate_camera
This node will determine the intrinsic camera parameters (such as focal length) as well as the distortion coefficients for the camera-lense combination. These parameters are then written to a file (manually) and are loaded automatically when launching the each camera.

# Python Scripts (scripts/)
## Active ROS Nodes
### coprocessor_serial
### pneumatics.py
## Utilities
### riptide_rules
### add_rule
