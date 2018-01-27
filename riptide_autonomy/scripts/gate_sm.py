#!/usr/bin/env python

import rospy
import smach
import smach_ros

gate_sm = smach.StateMachine(outcomes=['entered_qualify_gate','entered_casino_gate','exited_qualify_gate'],
                            input_keys=['prev_completion_in',
                                        'mission_type_in'],
                            output_keys=['prev_completion_out'])

#Remap userdata to each action state via goal_slots
#Remap userdata from an action state to the state machine's userdata via result_slots
with gate_sm:
    #Define goal callbacks
    def FindGoalCB(userdata, goal):
        find_goal = FindGoal()

        if userdata.mission_type_in == "qualify"
            if userdata.prev_completion_in == "nothing" || userdata.prev_completion_in == "circled_the_marker"
                find_goal.object = "qualify_gate"
        if userdata.mission_type_in == "competition"
            find_goal.object = "casino_gate"

    #NOTE: The GateGoalCB may not be necessary, since "object" could instead be
    #set within FindGoalCB. However, should the AlignmentGoal ever change,
    #a goal callback WILL be neccessary.
    def GateGoalCB(userdata, goal):
        alignment_goal = AlignmentGoal()

        if userdata.mission_type_in == "qualify"
            if userdata.prev_completion_in == "nothing" || userdata.prev_completion_in == "circled_the_marker"
                alignment_goal.object = "qualify_gate"
        if userdata.mission_type_in == "competition"
            alignment_goal.object = "casino_gate"

        alignment_goal.casino_color = "none"

    #Define result callbacks
    def GateResultCB(userdata, status, result):
        if status == GoalStatus.SECCEEDED:
            if self.userdata.mission_type_in == "qualify":
                if self.userdata.prev_completion_in == "nothing":
                    self.userdata_prev_completion_out = "entered_qualify_gate"
                    return 'entered_qualify_gate'

                elif self.userdata.prev_completion_in == "circled_the_marker":
                    self.userdata.prev_completion_out = "exited_qualify_gate"
                    return 'exited_qualify_gate'

            elif self.userdata.mission_type_in == "competition":
                if self.userdata.prev_completion_in == "nothing":
                    self.userdata.prev_completion_out = "entered_casino_gate"
                    return 'entered_casino_gate'

    #Add states
    smach.StateMachine.add('FIND_OBJECT',
                            SimpleActionState('FindServer',
                                                FindAction,
                                                goal_cb = FindGoalCB,
                                                input_keys = ['mission_type_in',
                                                              'prev_completion_in']),
                            transitions={'succeeded':'CONQUER_GATE'},
                            remapping={'mission_type_in':'mission_type_in',
                                       'prev_completion_in':'prev_completion_in'})
    smach.StateMachine.add('CONQUER_GATE',
                            SimpleActionState('AlignmentServer',
                                                AlignmentAction,
                                                goal_cb = GateGoalCB,
                                                result_cb = GateResultCB,
                                                output_keys = ['prev_completion_out']),
                            transitions={'entered_qualify_gate':'entered_qualify_gate',
                                        'entered_casino_gate':'entered_casino_gate',
                                        'exited_qualify_gate':'exited_qualify_gate'},
                            remapping={'mission_type_in':'mission_type_in',
                                        'prev_completion_in':'prev_completion_in',
                                        'prev_completion_out':'prev_completion_out'})

outcome = gate_sm.execute()
