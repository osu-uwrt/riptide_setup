---
description: >-
  List of ROS Topics and their uses. To bring these up launch
  /riptide_bringup/launch/local_bringup.launch
---

# Riptide ROS Topics

## ROS Topics:

### Align Topics

/align/cancel: time stamp string id

/align/feedback 

/align/goal 

/align/result 

/align/status 

### Alignment Topics

/alignment\_controller/parameter\_descriptions 

/alignment\_controller/parameter\_updates 

### Arc Topics

/arc/cancel 

/arc/feedback 

/arc/goal 

/arc/result 

/arc/status 

### Attitude Controller Topics

/attitude\_controller/parameter\_descriptions 

/attitude\_controller/parameter\_updates 

### Buoy Task Topics

/buoy\_task/cancel /buoy\_task/feedback 

/buoy\_task/goal /buoy\_task/result 

/buoy\_task/status /calibrate/cancel 

### Calibrate Topics

/calibrate/feedback 

/calibrate/goal 

/calibrate/result 

/calibrate/status 

### Calibrate Topics

/command/alignment 

/command/arm 

/command/camera 

/command/depth 

/command/drop 

/command/fire 

/command/force\_depth 

/command/force\_x 

/command/force\_y 

/command/force\_z 

/command/moment 

/command/net\_load 

/command/pitch 

/command/pwm 

/command/roll 

/command/thrust 

/command/x 

/command/y 

/command/yaw 

### Controls Topics

/controls/reset 

### Darknet Topics

/darknet\_ros/bounding\_boxes 

/darknet\_ros/detection\_image 

/darknet\_ros/image\_hud 

/darknet\_ros/image\_hud/compressed 

/darknet\_ros/image\_hud/compressed/parameter\_descriptions 

/darknet\_ros/image\_hud/compressed/parameter\_updates 

/darknet\_ros/input\_image 

### Decap Topics

/decap\_task/cancel 

/decap\_task/feedback 

/decap\_task/goal 

/decap\_task/result 

/decap\_task/status 

### Depth Controller Topics

/depth\_controller/parameter\_descriptions 

/depth\_controller/parameter\_updates 

### Downward Topics

/downward/image\_hud 

/downward/image\_hud/compressed 

/downward/image\_hud/compressed/parameter\_descriptions 

/downward/image\_hud/compressed/parameter\_updates 

/downward/image\_rect\_color 

### Task Topics

/expose\_task/cancel 

/expose\_task/feedback 

/expose\_task/goal 

/expose\_task/result 

/expose\_task/status 

/garlic\_task/cancel 

/garlic\_task/feedback 

/garlic\_task/goal 

/garlic\_task/result 

/garlic\_task/status 

/gate\_maneuver/cancel 

/gate\_maneuver/feedback 

/gate\_maneuver/goal 

/gate\_maneuver/result 

/gate\_maneuver/status 

/gate\_task/cancel 

/gate\_task/feedback 

/gate\_task/goal 

/gate\_task/result 

/gate\_task/status 

### Get Distance Topics

/get\_distance/cancel 

/get\_distance/feedback 

/get\_distance/goal 

/get\_distance/result 

/get\_distance/status 

### Got To Depth Topics

/go\_to\_depth/cancel 

/go\_to\_depth/feedback 

/go\_to\_depth/goal 

/go\_to\_depth/result 

/go\_to\_depth/status 

### Go to Finals Topics

/go\_to\_finals/cancel 

/go\_to\_finals/feedback 

/go\_to\_finals/goal 

/go\_to\_finals/result 

/go\_to\_finals/status 

### Go to Pitch Topics

/go\_to\_pitch/cancel 

/go\_to\_pitch/feedback 

/go\_to\_pitch/goal 

/go\_to\_pitch/result 

/go\_to\_pitch/status 

### Go to Roll Topics

/go\_to\_roll/cancel 

/go\_to\_roll/feedback 

/go\_to\_roll/goal 

/go\_to\_roll/result 

/go\_to\_roll/status 

### Go to Yaw Topics

/go\_to\_yaw/cancel 

/go\_to\_yaw/feedback 

/go\_to\_yaw/goal 

/go\_to\_yaw/result 

/go\_to\_yaw/status 

### Linear Controller Topics

/linear\_controller/parameter\_descriptions 

/linear\_controller/parameter\_updates 

### Move Distance Topics

/move\_distance/cancel 

/move\_distance/feedback 

/move\_distance/goal 

/move\_distance/result 

/move\_distance/status 

### Prequalify Topics

/prequalify/cancel 

/prequalify/feedback 

/prequalify/goal 

/prequalify/result 

/prequalify/status 

### Properties Topics

/properties/cob 

### Rosout Topics

/rosout 

/rosout\_agg 

### Search Topics

/search/cancel 

/search/feedback 

/search/goal 

/search/result 

/search/status 

### State Topics

/state/bboxes 

/state/depth 

/state/dvl 

/state/imu 

/state/object 

/state/switches 

### Status Topics

/status/controls/depth 

### Stereo Topics

/stereo/left/image\_hud 

/stereo/left/image\_hud/compressed 

/stereo/left/image\_hud/compressed/parameter\_descriptions 

/stereo/left/image\_hud/compressed/parameter\_updates 

/stereo/left/image\_rect\_color 

### Thruster Controller Topics

/thruster\_controller/parameter\_descriptions 

/thruster\_controller/parameter\_updates 

### Wait Topics

/wait/cancel 

/wait/feedback 

/wait/goal 

/wait/result 

/wait/status

#### PARAMETERS

* /calibrate/properties\_file: /home/osu/osu-uwr...
* /command\_combinator/max\_x\_force: 50.0
* /command\_combinator/max\_x\_moment: 15.0
* /command\_combinator/max\_y\_force: 50.0
* /command\_combinator/max\_y\_moment: 30.0
* /command\_combinator/max\_z\_force: 60.0
* /command\_combinator/max\_z\_moment: 40.0
* /depth\_controller/PID\_IIR\_LPF\_bandwidth: 17.0
* /depth\_controller/antiwindup: False
* /depth\_controller/d: 0.0
* /depth\_controller/i: 0.0
* /depth\_controller/i\_clamp\_max: 5.0
* /depth\_controller/i\_clamp\_min: 0.0
* /depth\_controller/max\_depth: 5.0
* /depth\_controller/max\_depth\_error: 1.0
* /depth\_controller/p: 60.0
* /depth\_controller/sensor\_rate: 22.0
* /pwm\_controller/TYPE0/MIN\_THRUST: 0.2
* /pwm\_controller/TYPE0/NEG\_THRUST/SLOPE: 8.54
* /pwm\_controller/TYPE0/NEG\_THRUST/YINT: 1444
* /pwm\_controller/TYPE0/POS\_THRUST/SLOPE: 6.66
* /pwm\_controller/TYPE0/POS\_THRUST/YINT: 1552
* /pwm\_controller/TYPE0/STARTUP\_THRUST: 1.5
* /pwm\_controller/TYPE0/SU\_NEG\_THRUST/SLOPE: 28.6
* /pwm\_controller/TYPE0/SU\_NEG\_THRUST/YINT: 1480
* /pwm\_controller/TYPE0/SU\_POS\_THRUST/SLOPE: 22.5
* /pwm\_controller/TYPE0/SU\_POS\_THRUST/YINT: 1523
* /pwm\_controller/TYPE1/MIN\_THRUST: 0.2
* /pwm\_controller/TYPE1/NEG\_THRUST/SLOPE: -8.54
* /pwm\_controller/TYPE1/NEG\_THRUST/YINT: 1552
* /pwm\_controller/TYPE1/POS\_THRUST/SLOPE: -6.66
* /pwm\_controller/TYPE1/POS\_THRUST/YINT: 1444
* /pwm\_controller/TYPE1/STARTUP\_THRUST: 1.5
* /pwm\_controller/TYPE1/SU\_NEG\_THRUST/SLOPE: -28.6
* /pwm\_controller/TYPE1/SU\_NEG\_THRUST/YINT: 1516
* /pwm\_controller/TYPE1/SU\_POS\_THRUST/SLOPE: -22.5
* /pwm\_controller/TYPE1/SU\_POS\_THRUST/YINT: 1473
* /pwm\_controller/properties\_file: /home/osu/osu-uwr...
* /rosdistro: kinetic
* /rosversion: 1.12.14
* /thruster\_controller/properties\_file: /home/osu/osu-uwr...

#### NODES / 

align \(riptide\_controllers/align.py\) 

alignment\_controller \(riptide\_controllers/alignment\_controller.py\) 

arc \(riptide\_controllers/arc.py\) 

attitude\_controller \(riptide\_controllers/attitude\_controller.py\) 

buoy\_task \(riptide\_autonomy/buoyTask.py\) 

calibrate \(riptide\_controllers/calibrate.py\) 

command\_combinator \(riptide\_controllers/command\_combinator\)

darkent\_processor \(riptide\_vision/darknet\_processor.py\) 

decap\_task \(riptide\_autonomy/decapTask.py\) 

depth\_controller \(riptide\_controllers/depth\_controller\) 

expose\_task \(riptide\_autonomy/exposeTask.py\) 

garlic\_task \(riptide\_autonomy/garlicTask.py\) 

gate\_maneuver \(riptide\_controllers/gateManeuver.py\) 

gate\_task \(riptide\_autonomy/gateTask.py\) 

get\_distance \(riptide\_controllers/getDistance.py\)

go\_to\_depth \(riptide\_controllers/goToDepth.py\)

go\_to\_finals \(riptide\_autonomy/goToFinals.py\)

go\_to\_pitch \(riptide\_controllers/goToPitch.py\)

go\_to\_roll \(riptide\_controllers/goToRoll.py\)

go\_to\_yaw \(riptide\_controllers/goToYaw.py\) 

hud \(riptide\_vision/hud\) 

linear\_controller \(riptide\_controllers/linear\_controller.py\) 

move\_distance \(riptide\_controllers/moveDistance.py\) 

prequalify \(riptide\_autonomy/prequalify.py\) 

pwm\_controller \(riptide\_controllers/pwm\_controller\) 

search \(riptide\_autonomy/search.py\) 

thruster\_controller \(riptide\_controllers/thruster\_controller\) 

wait \(riptide\_controllers/wait.py\)



