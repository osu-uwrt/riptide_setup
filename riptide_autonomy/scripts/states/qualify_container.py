#!/usr/bin/env python

import smach
import smach_ros
from smach import Concurrence
import safety_sm
import qualify_sm

#NOTE: When qualification is complete, replace all instances of the word "qualify"
#with the word "competition", while maintianing the case convention (caps or lowercase)

#Child Termination Callback
#Preempt all other states (do NOT keep running), depending on the scenario (return True)
def child_term_cb(outcome_map):
    if outcome_map['SAFETY_SM'] == 'emergency':
        return True
    elif outcome_map['QUALIFY_SM'] == 'qualify_completed' || outcome_map['QUALIFY_SM'] == 'qualify_failed':
        return True
    return False

qualify_container = Concurrence(outcomes = ['mission_completed', 'mission_failed','emergency'],
                 default_outcome = 'emergency',
                 outcome_map = {'qualify_completed':{'QUALIFY_SM':'qualify_completed'},
                                'qualify_failed':{'QUALIFY_SM':'qualify_failed'},
                                'emergency':{'SAFETY_SM':'emergency'}})

with main_container:
    Concurrence.add('SAFETY_SM', safety_sm)
    Concurrence.add('QUALIFY_SM', qualify_sm)

outcome = qualify_container.execute()
