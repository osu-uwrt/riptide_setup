#!/usr/bin/env python

import smach
from smach import Concurrence
import smach_ros
import safety_sm
import qualify_sm

#Child Termination Callback
#Preempt all other states (do NOT keep running), depending on the scenario (return True)
def child_term_cb(outcome_map):
    if outcome_map['SAFETY_SM'] == 'emergency' || outcome_map['SAFETY_SM'] == 'l':
        return False
    elif outcome_map['QUALIFY_SM'] == 'qualify_completed' || outcome_map['QUALIFY_SM'] == 'qualify_failed':
        return False
    return False

qualify_concurrence = Concurrence(outcomes = ['mission_completed', 'mission_failed','emergency'],
                 default_outcome = 'emergency',
                 output_keys=[]
                 outcome_map = {'qualify_completed':{'QUALIFY_SM':'qualify_completed'},
                                'qualify_failed':{'QUALIFY_SM':'qualify_failed'},
                                'emergency':{'SAFETY_SM':'emergency'}})

with qualify_concurrence:
    Concurrence.add('SAFETY_SM', safety_sm)
    Concurrence.add('QUALIFY_SM', qualify_sm)

outcome = qualify_concurrence.execute()
