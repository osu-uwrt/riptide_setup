#!/usr/bin/env python

################################
# SMACH State Machine Template #
################################

###########
# IMPORTS #
###########
# Include necessary libraries and your action and goal messages.

import roslib; roslib.load_manifest('riptide_autonomy')
import rospy
import smach
import smach_ros
import actionlib

from riptide_autonomy.msg import TestAction, TestGoal

#####################
# STATE DEFINITIONS #
#####################
# Define states that are not "Simple Action States" in here.

# Example:
class Foo(smach.State):
	# Define intitialization function. Runs on state creationg.
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed']);
	
	#Define execute function. Runs when state is active.
	def execute(self, userdata):
		return 'success';

########
# MAIN #
########
# Entry point for the state machine.

def main()
	rospy.init_node('STATE_MACHINE_NAME');
	
	# Create the state machine
	sm0 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'failed']);
	# Add states
	with sm0:
		#  Normal state
		smach.StateMachine.add('FOO', Foo(), transitions={'succeeded':'succeeded', 'failed':'failed'});
		
		# Simple Action State
		smach.StateMachine.add('TEST_ACTION', smach_ros.SimpleActionState('TestAction', TestAction, goal = 0), transitions{'succeeded:succeeded', 'aborted':'failed'});
		
	# Run the state machine
	outcome = sm.execute();
	
if __name__ == '__main__':
	main();
