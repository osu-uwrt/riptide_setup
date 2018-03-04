#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs import Constants

class SafetyMonitor(State):
    def __init__(self):
        State.__init__(self, outcome=['emergency', 'safe'],
                        input_keys=[],
                        output_keys=[])

    def execute(self, userdata):
        #copro_sub = rospy.Subscriber()

safety_sm = StateMachine(outcomes=['emergency', 'safe'],
                        output_keys=[])
safety_sm.userdata.safety_state = STATE_SAFE

# Things to monitor (0 = false, 1 = true)
# Kill Switch
# Bat voltages (cannot go low), current voltages (cannot spike)
# Converter voltages (cannot go low), converter current (cannot spike)
# Humidity sensor (lowest priority)

with safety_sm:
    StateMachine.add('SAFETY_MONITOR_SM', SafetyMonitor(),
                        transitions={'emergency':'emergency',
                                    'safe':'safe'},
                        remapping={})

safety_sm.execute()
