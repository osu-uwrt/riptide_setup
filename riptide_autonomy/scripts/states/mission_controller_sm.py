#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach_ros
from riptide_msgs import Constants
import mission_concurrence_sm

class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=['auv_idled'],
                        input_keys=[],
                        output_keys=[])
        #Initialize

    def execute(self, userdata):
        #Send message to controllers to reset themselves

mission_controller = StateMachine(outcomes = ['auv_idled'],
                                            input_keys=[],
                                            output_keys=[])

with mission_controller:
    StateMachine.add('MISSION_CONCURRENCE_SM', mission_concurrence_sm,
                    transitions={'mission_completed':'IDLE',
                                'mission_failed':'IDLE',
                                'kill_switch_disengaged':'MISSION_CONCURRENCE_SM'}
                    remapping={})
    StateMachine.add('IDLE', Idle(),
                    transitions={'auv_idled':'auv_idled'},
                    remapping={})

mission_controller.execute()
