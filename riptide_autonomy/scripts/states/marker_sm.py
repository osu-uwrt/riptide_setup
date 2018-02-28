#!/usr/bin/env python

import rospy
import smach
import smach_ros
import qualify_sm

def main():
    marker_sm = smach.StateMachine(outcomes=['circled_the_marker'],
                                output_keys=[])

    #Remap userdata to each action state via goal_slots
    #Remap userdata from an action state to the state machine's userdata via result_slots
    with marker_sm:
        def CircleMarkerResultCB(userdata, status, result):
            if status == GOAL_STATUS.SUCCEEDED:
                return 'succeeded'

        smach.StateMachine.add('FIND_MARKER',
                                SimpleActionState('FindServer',
                                                    FindMarkerAction),
                                transitions={'succeeded':'MARKER_ALIGNMENT'})
        smach.StateMachine.add('MARKER_ALIGNMENT',
                                SimpleActionState('AlignmentServer',
                                                    MarkerAlignmentAction,
                                                    result_slots=[]),
                                transitions={'succeeded':'CIRCLE_MARKER'},
                                remapping={})
        smach.StateMachine.add('CIRCLE_MARKER',
                                SimpleActionState('DynamicAlignmentServer',
                                                    CircleMarkerAction,
                                                    result_cb=CircleMarkerResultCB,
                                                    output_keys=[]),
                                transitions={'succeeded':'GATE_SM'},
                                remapping={})

    outcome = marker_sm.execute()

if __name__ == "__main__"
    main()
