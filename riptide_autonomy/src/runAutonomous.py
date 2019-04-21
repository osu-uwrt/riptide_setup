#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import roslib

class runAutonomous(smach.State) :
    def __init__(self): 
        smach.State.__init__(self, outcomes=['Gate Task','Path Markers', 'Nav', 'Buoy', 'Bin', 'Stake', 'Octagon', 'Error' ])
           