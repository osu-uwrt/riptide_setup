#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs import Constants, SwitchState

class KillSwitchMonitor(State):
    kill_switch_status = 0
    last_kill_switch_status = 0
    kill_switch_engage_time = 0

    def __init__(self):
        State.__init__(self, outcome=['kill_switch_engaged','kill_switch_disengaged'],
                        input_keys=[],
                        output_keys=['kill_switch_status',
                                    'kill_switch_engage_time'])
        rospy.init_node('kill_switch_monitor')
        copro_sub = rospy.Subscriber("/state/switches", callback)

    def execute(self, userdata):
        userdata.kill_switch_status = kill_switch_status
        userdata.kill_switch_engage_time = kill_switch_engage_time

        if kill_switch_status == SAFETY_KILL_SWITCH_ENGAGED:
            return 'kill_switch_engaged'
        elif kill_switch_status == SAFETY_KILL_SWITCH_DISENGAGED:
            # Call service to reset controllers if thrusters become inactive
            rospy.wait_for_service('reset_controllers')
            reset_srv = rospy.service('reset_controllers', ResetControllers)
            reset_result = reset_srv(REQUEST_RESET_CONTROLLERS)

            return 'kill_switch_disengaged'

    def callback(data):
        # Get current thruster status
        if data.kill == True:
            kill_switch_status = SAFETY_KILL_SWITCH_ENGAGED
        elif data.kill == False:
            kill_switch_status = SAFETY_KILL_SWITCH_DISENGAGED

        #Get time of switch engagement or disengagement
        if last_kill_switch_status ~= kill_switch_status:
            kill_switch_engage_time = rospy.Time.now().to_sec()

        last_kill_switch_status = kill_switch_status

kill_switch_sm = StateMachine(outcomes=['kill_switch_engaged', 'kill_switch_disengaged'],
                        output_keys=[thruster_status_sm])
kill_switch_sm.userdata.kill_switch_status_sm = 0
kill_switch_sm.userdata.kill_switch_engage_time_sm = 0

with safety_sm:
    StateMachine.add('KILL_SWITCH_MONITOR', KillSwitchMonitor(),
                        transitions={'kill_switch_disengaged':'kill_switch_disengaged'
                                    'kill_switch_engaged':'KILL_SWITCH_MONITOR'},
                        remapping={'kill_switch_status':'kill_switch_status_sm',
                                    'kill_switch_engage_time':'kill_switch_engage_time_sm'})

outcome = kill_switch_sm.execute()
