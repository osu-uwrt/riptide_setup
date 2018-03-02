The enforced style guide is the one used by flake8, when installed, you can
use the style checker by using the command 'flake8 file_name.py'


Core Dependencies
-Python 2.7
-OpenCV 2.4.9.1


What Each File Does
-hls_threshold_gui.py: It is a tool used to threshold a video using the HLS
colorspace, you can run it by typing 'python2 hls_threshold_gui.py'

-riptide_vision.py: A set of functions that can be used to help complete the
qualification maneuver

Notes:
*Some parts are not well documented

*detect_gate only attempts to cover cases were both side poles are 
detected, also even though it works, the current solution can be improved,
Parameters for houghlines can be fine tuned and right now code structure is
messy

*detect_pole currently makes the image smaller by a factor of 3


-frame_extracter.py: A function that can extract frames from a video, this 
file does not follow the code dependency requirements

-example.py: A small script to test run riptide_vision.py

*Description needs to be improved

Description: ...
