# The Riptide Hardware Package
This package handles the hardware-software interface with all of our AUVs sensors. Our AUV is equipped with:
* 1x [LordMicrostrain IMU 3DM-GX4](https://www.microstrain.com/inertial/3dm-gx4-25)
* 1x [BlueRobotics Bar30](https://www.bluerobotics.com/store/sensors-sonars-cameras/sensors/bar30-sensor-r1/) depth sensor
* 2x [Point Grey Blackfly USB 3.0](https://www.ptgrey.com/blackfly-13-mp-color-usb3-vision-sony-icx445) cameras
* 4x hydrophones, creating a single acoustics system

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
