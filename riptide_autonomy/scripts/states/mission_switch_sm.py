#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs import Constants
import qualify_sm

class MissionSwitchMonitor(State):
    qualify_switch_status = 0

    def __init__(self):
        State.__init__(self, outcomes=['qualify_switch_activated',
                                        'mission_switch_activated',
                                        'no_switch_activated',],
                        input_keys=[],
                        output_keys=[])
        rospy.init_node('mission_switch_monitor')
        copro_sub = rospy.Subscriber("/state/switches", callback)

    def execute(self, userdata):
        userdata.qualify_switch_status = qualify_switch_status
        userdata.mission_switch_status = mission_switch_status

        if qualify_switch_status == STATUS_ACTIVATED:
            return 'qualify_switch_activated'
        if mission_switch_status == STATUS_ACTIVATED:
            return 'mission_switch_activated'
        elif qualify_switch_status == STATUS_DEACTIVATED && mission_switch_status == STATUS_DEACTIVATED:
            return 'no_switch_activated'

    def callback(data):
        # Get current mission switch status
        if data.sw1 == True && data.kill == True:
            qualify_switch_status = STATUS_ACTIVATED
        elif data.sw2 == True && data.kill == True:
            mission_switch_status = STATUS_ACTIVATED
        else
            qualify_switch_status = STATUS_DEACTIVATED
            mission_switch_status = STATUS_DEACTIVATED


mission_switch_sm = StateMachine(outcomes=['mission_completed', 'mission_failed'],
                                input_keys=[],
                                output_keys=[])
with missim_switch_sm:
    StateMachine.add('MISSION_SWITCH_MONITOR', MissionSwitchMonitor(),
                    transitions={'no_switch_activated':'MISSION_SWITCH_MONITOR',
                                'qualify_switch_activated':'QUALIFY_SM',
                                'mission_switch_activated':'QUALIFY_SM'},
                    remapping={'thruster_status':'thruster_status_sm'})
    StateMachine.add('QUALIFY_SM', 'qualify_sm',
                    transitions={},
                    remapping={})
    #StateMachine.add('MISSION_SM', 'mission_sm',
    #                transitions={},
    #                remapping={})
missim_switch_sm.execute()
