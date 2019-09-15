---
description: riptide_vision
---

# Vision

This package handles all-things image processing \(except for YOLO, which is handled separately\).

## Source Code \(src/\)

### Active ROS Nodes

#### hud

This node reads in camera footage and adds a heads-up-display \(split into two sections above the image\) so we can view critical state information while viewing camra footage. The upper section displays the state and commands for: roll, pitch, yaw, and depth. The lower section displays the state and commands for: linear acceleration.

* Input Topics: //image\_undistorted, or /darknet\_ros/detection\_image
* Output Topics: //image\_hud, or /darknet\_ros/image\_hud

**yolo\_processor**

This node reads in the bounding box data from darknet and outputs the same type of message, but with bounding box info pertaining ONLY to the desired task at hand.

* Input Topics: /darknet\_ros/bounding\_boxes, /task/info
* Output Topics: /task/bboxes, /task/low\_detections

**object\_processor**

This node reads in data from yolo\_processor and outputs specific information about the bounding box \(width, height, and center\) for the alignment\_controller. Without known our position, we were forced to create an controller that aligns the vehicle to a detected object.

* Input Topics: /task/bboxes, /stereo/left/image\_undistorted, /task/info, /command/alignment
* Output Topics: /state/object

### Utility ROS Nodes

#### extract\_video

This node converts video footage from a ROS bag file to a .avi file. The launch file contains all of the required parameters, which are meant to be changed on a case-by-case basis.

* Input Topics: /

**darknet\_sim**

This node was created to publish "dummy" darknet\_ros data for the purposes of testing the yolo\_processor and object\_processor nodes. This node reads in the "dummy" data from a file called `tasks_sim.yaml` in the cfg/ folder.

* Output Topics: /darknet\_ros/bounding\_boxes

