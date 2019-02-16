# The Riptide Teleop Package
This package allows us to control our vehicle via PS3 controller.

# Source Code (src/)
## Active ROS Nodes
### ps3_controller
This node interfaces with a PS3 controller to give us extensive control of our vehicle. We can set attitude commands (roll, pitch, and yaw), as well as depth commands. For translational motion, we can move one of the joysticks to set acceleration commands. Since we have a forward-facing and downward-facing camera, pressing the "PS3" button will toggle which camera footage YOLO should proces. Last, pressing the "X" will kill power to all thrusters.
* Input Topics: /joy, /state/depth, /state/imu
* Output Topics: /command/attitude, /command/depth, /command/accel_x, /command/accel_y, /command/accel_z, /command/ps3_plane, /command/pneumatics, /controls/reset
