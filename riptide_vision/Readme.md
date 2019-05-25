# The Riptide Vision Package
This package handles all-things image processing (except for YOLO, which is handled separately).

## Source Code (src/)

### Active ROS Nodes
#### hud
This node reads in camera footage and adds a heads-up-display (split into two sections above the image) so we can view critical state information while viewing camra footage. The upper section displays the state and commands for: roll, pitch, yaw, and depth. The lower section displays the state and commands for: linear acceleration.
* Input Topics: /<camera_name>/image_undistorted, or /darknet_ros/detection_image
* Output Topics: /<camera_name>/image_hud, or /darknet_ros/image_hud
#### yolo_processor
This node reads in the bounding box data from darknet and outputs the same type of message, but with bounding box info pertaining ONLY to the desired task at hand.
* Input Topics: /darknet_ros/bounding_boxes, /task/info
* Output Topics: /task/bboxes, /task/low_detections
#### object_processor
This node reads in data from yolo_processor and outputs specific information about the bounding box (width, height, and center) for the alignment_controller. Without known our position, we were forced to create an controller that aligns the vehicle to a detected object.
* Input Topics: /task/bboxes, /forward/image_undistorted, /task/info, /command/alignment
* Output Topics: /state/object

### Utility ROS Nodes
#### extract_video
This node converts video footage from a ROS bag file to a .avi file. The launch file contains all of the required parameters, which are meant to be changed on a case-by-case basis.
* Input Topics: /<some_input_topic>
#### darknet_sim
This node was created to publish "dummy" darknet_ros data for the purposes of testing the yolo_processor and object_processor nodes. This node reads in the "dummy" data from a file called `tasks_sim.yaml` in the cfg/ folder.
* Output Topics: /darknet_ros/bounding_boxes
