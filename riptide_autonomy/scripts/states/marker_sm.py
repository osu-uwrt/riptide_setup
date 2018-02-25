#!/usr/bin/env python

import rospy
import smach
import smach_ros
import qualify_sm

def main():
    marker_sm = smach.StateMachine(outcomes=['circled_the_marker'],
                                output_keys=['prev_completion_out'])
    marker_sm.userdata.depth = 0
    marker_sm.userdata.initial_euler_rpy = [0,0,0]

    #Remap userdata to each action state via goal_slots
    #Remap userdata from an action state to the state machine's userdata via result_slots
    with marker_sm:
        def CircleMarkerResultCB(userdata, status, result):
            if status == GOAL_STATUS.SUCCEEDED:
                #Adjust the 'prev_completion' variable
                userdata.prev_completion_out = "circled_the_marker"
                return 'succeeded'

        smach.StateMachine.add('FIND_MARKER',
                                SimpleActionState('FindServer',
                                                    FindMarkerAction),
                                transitions={'succeeded':'MARKER_ALIGNMENT'})
        smach.StateMachine.add('MARKER_ALIGNMENT',
                                SimpleActionState('AlignmentServer',
                                                    MarkerAlignmentAction,
                                                    result_slots=['depth', 'euler_rpy']),
                                transitions={'succeeded':'CIRCLE_MARKER'},
                                remapping={'depth':'depth', 'euler_rpy':'euler_rpy'})
        smach.StateMachine.add('CIRCLE_MARKER',
                                SimpleActionState('DynamicAlignmentServer',
                                                    CircleMarkerAction,
                                                    result_cb=CircleMarkerResultCB,
                                                    output_keys=['prev_completion_out']),
                                transitions={'succeeded':'GATE_SM'},
                                remapping={'prev_completion_out':'prev_completion_out'})

    outcome = marker_sm.execute()

if __name__ == "__main__"
    main()
