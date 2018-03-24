#!/usr/bin/env python

import rospy
from smach import Concurrence
import smach_ros
from riptide_msgs import Constants
import computer_controller_sm
import mission_controller_sm

#Child Termination Callback
#Preempt all other states (do NOT keep running), depending on the scenario (return True)
def child_term_cb(outcome_map):
    if outcome_map['COMPUTER_CONTROLLER_SM'] == 'computer_shutdown':
        return True
    elif outcome_map['COMPUTER_CONTROLLER_SM'] == 'computer_restart':
        return True
    elif outcome_map['MISSION_CONTROLLER_SM'] == 'mission_completed':
        return True
    return False

auv_controller = Concurrence(outcomes = ['computer_shutdown', 'computer_restart',
                                            'mission_completed'],
                 default_outcome = 'mission_completed',
                 output_keys=[]
                 outcome_map = {'computer_shutdown':{'COMPUTER_CONTROLLER_SM':'computer_shutdown'},
                                'computer_restart':{'COMPUTER_CONTROLLER_SM':'computer_restart'},
                                'mission_completed':{'MISSION_CONTROLLER_SM':'mission_completed'}})

with auv_controller:
    Concurrence.add('COMPUTER_CONTROLLER_SM', computer_controller_sm,
                    remapping={})
    Concurrence.add('MISSION_CONTROLLER_SM', mission_controller_sm,
                    remapping={})

auv_controller.execute()
