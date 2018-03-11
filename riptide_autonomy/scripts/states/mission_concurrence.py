#!/usr/bin/env python

import rospy
from smach import Concurrence
import smach_ros
from riptide_msgs import Constants
import mission_switch_sm
import kill_switch__sm
import saftey_sm

#Child Termination Callback
#Preempt all other states (do NOT keep running), depending on the scenario (return True)
def child_term_cb(outcome_map):
    if outcome_map['SAFETY_SM'] == 'emergency':
        return True
    elif outcome_map['MISSION_SM'] == 'mission_completed' || outcome_map['MISSION_SM'] == 'mission_failed':
        return True
    return False

mission_concurrence = Concurrence(outcomes = ['mission_completed', 'mission_failed',
                                            'emergency', 'thrusters_deactivated'],
                 default_outcome = 'thrusters_deactivated',
                 output_keys=[]
                 outcome_map = {'misson_completed':{'MISSION_SM':'mission_completed'},
                                'mission_failed':{'MISSION_SM':'mission_failed'},
                                'emergency':{'SAFETY_SM':'emergency'}})
qualify_concurrence.userdata.thruster_status_cc = 0
qualify_concurrence.userdata.kill_switch_engage_time_cc = 0

with qualify_concurrence:
    Concurrence.add('MISSION_SWITCH_MONITOR_SM', mission_switch_monitor_sm,
                    remapping={'thruster_status_sm':'thruster_status_cc',
                                'kill_switch_engage_time_sm':'kill_switch_engage_time_cc'})
    Concurrence.add('SAFETY_SM', safety_sm,
                    remapping={})
    Concurrence.add('KILL_SWITCH_SM', kill_switch_sm,
                    remapping={'thruster_status_sm':'thruster_status_cc',
                                'kill_switch_engage_time_sm':'kill_switch_engage_time_cc'})

qualify_concurrence.execute()
