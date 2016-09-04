#!/usr/bin/env python

##############################
# RIPTIDE MAIN STATE MACHINE #
##############################

###########
# IMPORTS #
###########
# Include necessary libraries and your action and goal messages.

import roslib
import roslib.load_manifest('riptide_autonomy')
import rospy
import smach
import smach_ros
import actionlib

from riptide_msgs.msg import FindGateAction, FindGateGoal

#####################
# STATE DEFINITIONS #
#####################
# Define states that are not "Simple Action States" in here.
        
########
# MAIN #
########
# Entry point for the state machine.

# Define some constants


def main():
    rospy.init_node('Qualifying_Run_SMACH')

    FindGateTimeout = rospy.Duration(10.0)
    # FindGateGoal = FindGateGoal()

    # Create the state machine
    sm0 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'failed'])
    # Add states
    with sm0:

        # Find Gate Action State
        #smach.StateMachine.add('FindGate', smach_ros.SimpleActionState('FindGateAction', FindGateAction, goal=FindGateGoal, result_key='FindGateResult', exec_timeout=FindGateTimeout), transitions={'succeeded': 'succeeded', 'preempted': 'failed', 'aborted': 'failed'})
        
    # Run the state machine
    outcome = sm0.execute()

def batteryCB(data):
  if not isStarted:
    isStared = True
    main()
  
if __name__ == '__main__':
  isStarted = False
  rospy.Subscriber('state/battery', Bat, batteryCB)
