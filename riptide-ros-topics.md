---
description: >-
  List of ROS Topics and their uses. To bring these up launch
  /riptide_bringup/launch/local_bringup.launch
---

# Riptide ROS Topics

## ROS Topics:

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



