# The Riptide Controllers Package
This package handles vehicle controls (for maneuverabiltity) using a decoupled PID control system. Our AUV's actuators consist of 8x [BlueRobotics T200 Thrusters](https://www.bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/).

# Config Files (cfg/)
This folder contains a plethora of configuration files needed by the PID controllers (namely the Kp, Ki, and Kd constants), properties for each control node, location of each thruster relative to the CoM, and vehicle properties.

# Source Code (src/)
## Active ROS Nodes
### depth_controller
This node uses a PID controller to maintain our vehicle's depth.
* Input Topics: /state/depth, /state/imu, /command/depth, /controls/reset
* Output Topics: /command/accel_depth, /status/controls/depth
### attitude_controller
This node contains three nodes (rcpid, pcpid, and ycpid) that each use a PID controller to maintain our vehicle's orientation (roll, pitch, and yaw).
* Input Topics: /state/imu, /command/attitude, /controls/reset
* Output Topics: /command/accel_angular, /status/controls/angular
### alignment_controller
This node contains three nodes (surge, sway, and heave) that each use a PID controller to align our vehicle with a desired bounding box obtained from the camera footage.
* Input Topics: /state/object, /state/depth, /command/alignment, /task/info, /controls/reset 
* Output Topics: /command/accel_x, /command/accel_y, /command/depth, /status/controls/linear, 
### command_combinator
This node reads in all of the acceleration commands from the above PID controllers and combines the data into a single acceleration message used by our thruster controller.
* Input Topics: /command/accel_x, /command/accel_y, /command/accel_z, /command/accel_depth, /command/accel_angular
* Output Topics: /command/accel
### thruster_controller
This nodes uses Google's non-linear solver [ceres](http://ceres-solver.org) to simultaneously solve our 6DOF equations of motion for the required thruster forces. If "debug" is enabled, then it will also solve for the approximate location of the center of buoyancy (CoB) relative to the center of mass (CoM).
* Input Topics: /state/imu, /state/depth, /command/accel
* Output Topics: /command/thrust, /status/controls/thruster, (/debug/pos_buoyancy)
### pwm_controller
This node reads in data from the thruster controller and uses our thruster calibration file to determine what pwm value to send to the coprocessor for the correct force outputs.
* Input Topics: /command/thrust, /state/switches, /controls/reset
* Output Topics: /command/pwm
