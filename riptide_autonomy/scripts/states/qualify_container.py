#!/usr/bin/env python

import smach
from smach import StateMachine
from smach import State
import smach_ros
import qualify_concurrence
from riptide_msgs import Constants
from std_msgs import uint8
import subprocess

class EmergencyExit(State):
    def __init__(self):
        State.__init__(self, outcomes=['emergency'],
                        input_keys=[],
                        output_keys=[])

    def execute(self, userdata):
        error_pub = rospy.Publisher('state/emergency', uint8, queue_size=1)
        rospy.init_node('exit_error')
        rate = rospy.rate(100)

        while not rospy.is_shutdown():

        #subprocess.call(["shutdown", "-h", "now")

qualify_container = StateMachine(outcomes = ['emergency', 'qualify_completed','qualify_failed'],
                 input_keys=[],
                 output_keys=[])

with qualify_container:
    StateMachine.add('QUALIFY_CONCURRENCE', qualify_concurrence,
                    input_keys=[],
                    output_keys=[],
                    transitions={'emergency':'EMERGENCY_EXIT',
                                'qualify_completed':'IDLE',
                                'qualify_failed':'IDLE'})
    StateMachine.add('EMERGENCY_EXIT', EmergencyExit(),
                    input_keys=[])

qualify_container.execute()
rospy.spin()
