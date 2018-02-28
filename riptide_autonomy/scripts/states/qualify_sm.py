#!/usr/bin/env python

import rospy
import smach
import smach_ros
from riptide_msgs import Constants
import main_container

qualify_sm = smach.StateMachine(outcomes=['qualify_completed', 'qualify_failed'])

with qualify_sm:
    smach.StateMachine.add('QUALIFY_GATE_SM', qualify_gate_sm,
                            transitions={'entered_qualify_gate':'MARKER_SM',
                                        'exited_qualify_gate':'qualify_completed'},
                            remapping={})
    smach.StateMachine.add('MARKER_SM', marker_sm,
                            transitions={'circled_the_marker':'QUALIFY_GATE_SM'},
                            remapping={})
