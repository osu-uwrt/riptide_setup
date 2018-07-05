#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs.msg import Constants, SwitchState

class KillSwitchMonitor(State):
    kill_switch_status = STATUS_INIT

    def __init__(self):
        State.__init__(self, outcome=['kill_switch_activated','kill_switch_deactivated'],
                        input_keys=[],
                        output_keys=[])
        rospy.init_node('kill_switch_monitor')
        switch_sub = rospy.Subscriber("/state/switches", SwitchState, callback)

    def execute(self, userdata):
        if kill_switch_status == STATUS_ACTIVATED:
            return 'kill_switch_activated'
        else:
            return 'kill_switch_deactivated'

    def callback(data):
        # Get kill switch status
        if data.kill == True:
            kill_switch_status = STATUS_DEACTIVATED
        elif data.kill == False:
            kill_switch_status = STATUS_ACTIVATED

kill_switch_monitor_sm = StateMachine(outcomes=['kill_switch_activated'],
                                        input_keys=[],
                                        output_keys=[])

with master_switch_monitor_sm:
    StateMachine.add('KILL_SWITCH_MONITOR', KillSwitchMonitor(),
                        transitions={'kill_switch_activated':'kill_switch_activated',
                                    'kill_switch_deactivated':'KILL_SWITCH_MONITOR'},
                        remapping={})

outcome = kill_switch_monitor_sm.execute()
