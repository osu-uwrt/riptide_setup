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

from riptide_msgs.msg import BuoyAction, BuoyGoal

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
    rospy.init_node('Riptide_State_Machine')

    Default_Timeout = rospy.Duration(10.0)

    GreenBuoyGoal = BuoyGoal(color="green")

    # Create the state machine
    sm0 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'failed'])
    # Add states
    with sm0:

        # GO TO B
        smach.StateMachine.add('FindGreenBuoy', smach_ros.SimpleActionState('BuoyAction', TestAction, goal=GreenBuoyGoal, result_key='GreenBuoyResult',  output_keys=['GreenBuoyResult'], exec_timeout=Default_Timeout), transitions={'succeeded': 'succeeded', 'aborted': 'failed'})

    # Run the state machine
    outcome = sm0.execute()

if __name__ == '__main__':
    main()
