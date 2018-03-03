#!/usr/bin/env python

import rospy
import smach
import smach_ros
from riptide_msgs import Constants
import main_container

qualify_sm = smach.StateMachine(outcomes=['qualify_completed', 'qualify_failed'])
qualify_sm.userdata.riptideConstantOffset = 100

with qualify_sm:
    smach.StateMachine.add('QUALIFY_GATE_SM', qualify_gate_sm,
                            input_keys=['RCOffset'],
                            transitions={'entered_qualify_gate':'MARKER_SM',
                                        'exited_qualify_gate':'qualify_completed'},
                            remapping={'RCOffset':'riptideConstantOffset'})
    smach.StateMachine.add('MARKER_SM', marker_sm,
                            input_keys=['RCOffset'],
                            transitions={'circled_the_marker':'QUALIFY_GATE_SM'},
                            remapping={'RCOffset':'riptideConstantOffset'})
