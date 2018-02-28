#!/usr/bin/env python

import rospy
import smach
import smach_ros
from riptide_msgs import Constants

gate_sm = smach.StateMachine(outcomes=['entered_qualify_gate','exited_qualify_gate'],
                            input_keys=[],
                            output_keys=[])

#Remap userdata to each action state via goal_slots
#Remap userdata from an action state to the state machine's userdata via result_slots
with gate_sm:
    #Define goal callbacks
    def FindGoalCB(userdata, goal):
        find_goal = FindGoal()
        find_goal.task = TASK_QUALIFY_GATE

    def QualifyGateGoalCB(userdata, goal):
        alignment_goal = AlignmentGoal()
        alignment_goal.task = TASK_QUALIFY_GATE

    #Define result callbacks
    def QualifyGateResultCB(userdata, status, result):
    

    #Add states
    smach.StateMachine.add('FIND_OBJECT',
                            SimpleActionState('FindServer',
                                                FindAction,
                                                goal_cb = FindGoalCB,
                                                input_keys = []),
                            transitions={'succeeded':'QUALIFY_GATE'},
                            remapping={})
    smach.StateMachine.add('QUALIFY_GATE',
                            SimpleActionState('QualifyGateServer',
                                                QualifyGateAction,
                                                goal_cb = QualifyGateGoalCB,
                                                result_cb = QualifyGateResultCB,
                                                output_keys = []),
                            transitions={'entered_qualify_gate':'entered_qualify_gate',
                                        'exited_qualify_gate':'exited_qualify_gate'},
                            remapping={})

outcome = gate_sm.execute()
