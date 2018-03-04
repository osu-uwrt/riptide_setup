#!/usr/bin/env python

from smach import State, StateMachine
import smach_ros
from riptide_msgs import Constants
import qualify_concurrence
import subprocess

class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=['emergency'],
                        input_keys=[],
                        output_keys=[])

    def execute(self, userdata):
        rospy.init_node('idle')
        error_pub = rospy.Publisher('state/emergency', uint8, queue_size=1)

        rate = rospy.rate(100)

        while not rospy.is_shutdown():

        #subprocess.call(["shutdown", "-h", "now")

qualify_container = StateMachine(outcomes = ['emergency', 'qualify_completed','qualify_failed'],
                 input_keys=[],
                 output_keys=[])
qualify_container.userdata.qualify_state = STATE

with qualify_container:
    StateMachine.add('QUALIFY_CONCURRENCE', qualify_concurrence,
                    transitions={'emergency':'EMERGENCY_EXIT',
                                'qualify_completed':'IDLE',
                                'qualify_failed':'IDLE'})
    StateMachine.add('IDLE', Idle(),
                    input_keys=[])

qualify_container.execute()
rospy.spin()
