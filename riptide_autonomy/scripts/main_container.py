#!/usr/bin/env python

import smach
import smach_ros
from smach import Concurrence
import safety_sm
import qualify_sm

#When qualification is complete, replace all instances of the word "qualify"
#with the word "mission", while maintianing the case convention (caps or lowercase)

#Child Termination Callback
#Preempt all other states (do NOT keep running), depending on the scenario (return True)
def child_term_cb(outcome_map):
    if outcome_map['SAFETY_SM'] == 'emergency':
        return True
    elif outcome_map['MISSION_SM'] == 'succeeded_MS' || outcome_map['MISSION_SM'] == 'failed_MS':
        return True
    return False

def main():
    #rospy.init_node('state_machine')

    main_container = Concurrence(outcomes = ['mission_completed', 'mission_failed','emergency'],
                     default_outcome = 'emergency',
                     outcome_map = {'mission_completed':{'QUALIFY_SM':'qualify_completed'},
                                    'mission_failed':{'QUALIFY_SM':'qualify_failed'},
                                    'emergency':{'SAFETY_SM':'emergency'}})

    with main_container:
        Concurrence.add('SAFETY_SM', safety_sm)
        Concurrence.add('QUALIFY_SM', qualify_sm)

    outcome = main_container.execute()

if __name__ == '__main__'
    main()
