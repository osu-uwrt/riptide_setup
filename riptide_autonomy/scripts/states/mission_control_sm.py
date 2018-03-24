#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs import Constants, SwitchState
import qualify_sm
#import mission_sm

class MissionSwitchMonitor(State):
    qualify_switch_status = 0
    mission_switch_status = 0

    def __init__(self):
        State.__init__(self, outcomes=['qualify_switch_activated',
                                        'mission_switch_activated',
                                        'no_switch_activated',],
                        input_keys=[],
                        output_keys=[])
        rospy.init_node('mission_switch_monitor')
        copro_sub = rospy.Subscriber("/state/switches", SwitchState, callback)

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

mission_control_sm = StateMachine(outcomes=['mission_completed', 'mission_failed'],
                                input_keys=[],
                                output_keys=[])

#Change second QUALIFY_SM to MISSION_SM when mission_sm is ready
with missim_control_sm:
    StateMachine.add('MISSION_SWITCH_MONITOR', MissionSwitchMonitor(),
                    transitions={'no_switch_activated':'MISSION_SWITCH_MONITOR',
                                'qualify_switch_activated':'QUALIFY_SM',
                                'mission_switch_activated':'MISSION_SWITCH_MONITOR'},
                    remapping={})
    StateMachine.add('QUALIFY_SM', 'qualify_sm',
                    transitions={'qualify_completed':'mission_completed'},
                    remapping={})
    #StateMachine.add('MISSION_SM', 'mission_sm',
    #                transitions={},
    #                remapping={})
missim_control_sm.execute()
