#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach_ros
from riptide_msgs import Constants

gate_sm = StateMachine(outcomes=['entered_qualify_gate','exited_qualify_gate'],
                            input_keys=[RCOffset],
                            output_keys=[])

#Remap userdata to each action state via goal_slots
#Remap userdata from an action state to the state machine's userdata via result_slots
with gate_sm:
    #Define goal callbacks
    def FindGoalCB(userdata, goal):
        find_goal = FindGoal()
        find_goal.task = TASK_QUALIFY_GATE
        find_goal.RCOffset_in = userdata.RCOffset

    def QualifyGateGoalCB(userdata, goal):
        align_goal = AlignmentGoal()
        align_goal.task = TASK_QUALIFY_GATE
        align_goal.RCOffset_in = userdata.RCOffset

    #Define result callbacks
    def QualifyGateResultCB(userdata, status, result):
    #Output correct result - entered or exited qualify gate

    #Add states
    StateMachine.add('FIND_OBJECT',
                        SimpleActionState('FindServer',
                                            FindAction,
                                            goal_cb = FindGoalCB,
                                            input_keys = ['RCOffset_in']),
                        transitions={'succeeded':'QUALIFY_GATE'},
                        remapping={'RCOffset_in':'RCOffset'})
    StateMachine.add('QUALIFY_GATE',
                        SimpleActionState('QualifyGateServer',
                                            QualifyGateAction,
                                            goal_cb = QualifyGateGoalCB,
                                            result_cb = QualifyGateResultCB,
                                            input_keys = ['RCOffset']),
                        transitions={'entered_qualify_gate':'entered_qualify_gate',
                                    'exited_qualify_gate':'exited_qualify_gate'},
                        remapping={'RCOffset_in':'RCOffset'})

outcome = gate_sm.execute()
