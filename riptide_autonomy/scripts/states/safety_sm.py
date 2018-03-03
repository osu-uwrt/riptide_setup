#!/usr/bin/env python

import rospy
from smach import StateMachine
from smach import State
import smach_ros
from riptide_msgs import Constants
import main_container

class SafetyMonitor(State):
    def __init__(self):
        State.__init__(self, outcome=['emergency', 'safe','kill_switch_engaged'],
                        input_keys=[],
                        output_keys=[])

    def execute(self, userdata):
        #copro_sub = rospy.Subscriber()

safety_sm = StateMachine(outcomes=['emergency', 'safe', 'kill_switch_engaged'],
                        output_keys=[])

# Things to monitor (0 = false, 1 = true)
# Kill Switch
# Bat voltages (cannot go low), current voltages (cannot spike)
# Converter voltages (cannot go low), converter current (cannot spike)
# Humidity sensor (lowest priority)

with safety_sm:
    StateMachine.add('SAFETY_MONITOR_SM', SafetyMonitor(),
                        input_keys=[],
                        transitions={'emergency':'emergency',
                                    'safe':'safe',
                                    'kill_switch_engaged':'kill_switch_engaged'},
                        remapping={})
