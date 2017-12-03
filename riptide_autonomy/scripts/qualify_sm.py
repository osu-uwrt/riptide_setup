#!/usr/bin/env python

import rospy
import smach
import smach_ros
import main_container

def main():
    qualify_sm = smach.StateMachine(outcomes=['qualify_completed', 'qualify_failed'])
    qualify_sm.userdata.prev_completion = "nothing"
    qualify_sm.userdata.gate_type = "qualify"

    with qualify_sm:
        smach.StateMachine.add('GATE_SM', gate_sm,
                                transitions={'entered_qualify_gate':'MARKER_SM',
                                            'exited_qualify_gate':'qualify_completed'},
                                remapping={'prev_completion_in':'prev_completion',
                                            'gate_type_in':'gate_type',
                                            'prev_completion_out':'prev_completion'})
        smach.StateMachine.add('MARKER_SM', marker_sm,
                                transitions={'circled_the_marker':'GATE_SM'},
                                remapping={'prev_completion_out':'prev_completion'})

if __name__ == "__main__"
    main()
