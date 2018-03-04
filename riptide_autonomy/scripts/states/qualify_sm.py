#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach_ros
from riptide_msgs import Constants

qualify_sm = StateMachine(outcomes=['qualify_completed', 'qualify_failed'],
                        input_keys=['thruster_state_in'],
                        output_keys=[])
qualify_sm.userdata.riptideConstantsOffset = 100
qualify_sm.userdata.qualify_switch_engage_time = 0 #Set to 0 initially

with qualify_sm:

    StateMachine.add('QUAL"SWITCH_MONITOR', QualifySwitchMonitor(),
                    transitions={},
                    remapping={})
    StateMachine.add('QUALIFY_GATE_SM', qualify_gate_sm,
                        transitions={'entered_qualify_gate':'MARKER_SM',
                                    'exited_qualify_gate':'qualify_completed'},
                        remapping={'RCOffset':'riptideConstantsOffset'})
    StateMachine.add('MARKER_SM', marker_sm,
                        transitions={'circled_the_marker':'QUALIFY_GATE_SM'},
                        remapping={'RCOffset':'riptideConstantsOffset'})

qualify_sm.execute()
