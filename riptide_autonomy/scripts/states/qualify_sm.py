#!/usr/bin/env python

import rospy
import smach
import smach_ros
import main_container

qualify_sm = smach.StateMachine(outcomes=['qualify_completed', 'qualify_failed'])
qualify_sm.userdata.mission_type = "qualify" #The mission is to qualify
qualify_sm.userdata.prev_completion = "nothing" #Initialize to "nothing"
qualify_sm.userdata.casino_color = "none" #Initialize to "none"

with qualify_sm:
    smach.StateMachine.add('GATE_SM', gate_sm,
                            transitions={'entered_qualify_gate':'MARKER_SM',
                                        'exited_qualify_gate':'qualify_completed'},
                            remapping={'prev_completion_in':'prev_completion',
                                        'prev_completion_out':'prev_completion',
                                        'mission_type_in':'mission_type',
                                        'casino_color_in':'casino_color'})
    smach.StateMachine.add('MARKER_SM', marker_sm,
                            transitions={'circled_the_marker':'GATE_SM'},
                            remapping={'prev_completion_out':'prev_completion'})
