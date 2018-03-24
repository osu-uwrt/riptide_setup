#!/usr/bin/env python

import rospy
from smach import Concurrence
import smach_ros
from riptide_msgs import Constants
import switch_monitor_sm
import mission_control_sm
#import saftey_sm

#Child Termination Callback
#Preempt all other states (do NOT keep running), depending on the scenario (return True)
def child_term_cb(outcome_map):
    if outcome_map['MASTER_SWITCH_MONITOR_SM'] == 'master_switch_activated':
        return True
    elif outcome_map['MISSION_CONTROL_SM'] == 'mission_completed':
        return True
    elif outcome_map['MISSION_CONTROL_SM'] == 'mission_failed':
        return True
    #elif outcome_map['SAFETY_SM'] == 'emergency':
    #    return True

    return False

auv_concurrence = Concurrence(outcomes = ['mission_completed', 'mission_failed',
                                            'master_switch_activated', 'emergency'],
                 default_outcome = 'mission_completed',
                 output_keys=[]
                 outcome_map = {'misson_completed':{'MISSION_SWITCH_SM':'mission_completed'},
                                'mission_failed':{'MISSION_SWITCH_SM':'mission_failed'},
                                'kill_switch_disengaged':{'KILL_SWITCH_SM':'kill_switch_disengaged'}})
                                #'emergency':{'SAFETY_SM':'emergency'},

with auv_concurrence:
    Concurrence.add('SWITCH_MONITOR_SM', switch_monitor_sm,
                    remapping={})
    Concurrence.add('MISSION_CONTROL_SM', mission_control_sm,
                    remapping={})
    #Concurrence.add('SAFETY_SM', safety_sm,
    #                remapping={})


auv_concurrence.execute()
