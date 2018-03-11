#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs import Constants
import qualify_sm

class MissionSwitchMonitor(State):
    mission_switch_status = 0

    def __init__(self):
        State.__init__(self, outcomes=['qualify_switch_activated',
                                        'mission_switch_activated',
                                        'no_switch_activated',],
                        input_keys=[],
                        output_keys=['mission_switch_status'])
        rospy.init_node('mission_switch_monitor')
        copro_sub = rospy.Subscriber("/state/switches", callback)

    def execute(self, userdata):
        userdata.mission_switch_status = mission_switch_status

    def callback(data):
        # Get current mission switch status
        if data.sw1 == True:
            mission_switch_status = STATUS_ACTIVATED
        else if data.sw1 == False:
            mission_switch_status = STATUS_DEACTIVATED


mission_switch_sm = StateMachine(outcomes=['mission_completed', 'mission_failed'],
                                input_keys=[],
                                output_keys=[])
with missim_switch_sm:
    StateMachine.add('MISSION_SWITCH_MONITOR', MissionSwitchMonitor(),
                    transitions={},
                    remapping={})
    StateMachine.add('QUALIFY_SM', 'qualify_sm',
                    transitions={},
                    remapping={})
    #StateMachine.add('MISSION_SM', 'mission_sm',
    #                transitions={},
    #                remapping={})
missim_switch_sm.execute()
