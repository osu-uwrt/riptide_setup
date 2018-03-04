#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs import Constants, SwitchState

class KillSwitchMonitor(State):
    thruster_status = 0
    last_thruster_status = 0
    kill_switch_engage_time = 0

    def __init__(self):
        State.__init__(self, outcome=['kill_switch_engaged','kill_switch_dissengaged'],
                        input_keys=[],
                        output_keys=['thruster_status',
                                    'kill_switch_engage_time'])
        rospy.init_node('kill_switch_monitor')
        copro_sub = rospy.Subscriber("/state/switches", callback)

    def execute(self, userdata):
        userdata.thruster_status = thruster_status
        userdata.kill_switch_engage_time = kill_switch_engage_time

        # Call service to reset controllers if thrusters become inactive
        if userdata.thruster_status = STATE_INACTIVE
            rospy.wait_for_service('reset_controllers')
            reset_srv = rospy.service('reset_controllers', ResetControllers)
            reset_result = reset_srv(REQUEST_RESET_CONTROLLERS)

    def callback(data):
        # Get current thruster status
        if data.kill == True:
            thruster_status = STATUS_ACTIVE
        else if data.kill == False:
            thruster_status = STATUS_INACTIVE

        # Get time of switch engagement or disengagement
        if last_thruster_status ~= thruster_status:
            kill_switch_engage_time = rospy.Time.now().to_sec()

        last_thruster_status = thruster_status


kill_switch_sm = StateMachine(outcomes=['kill_switch_engaged', 'kill_switch_disengaged'],
                        output_keys=[thruster_status_sm])
kill_switch_sm.userdata.thruster_status_sm = 0
kill_switch_sm.userdata.kill_switch_engage_time_sm = 0

with safety_sm:
    StateMachine.add('KILL_SWITCH_MONITOR', KillSwitchMonitor(),
                        transitions={'kill_switch_disengaged':'KILL_SWITCH_MONITOR'
                                    'kill_switch_engaged':'KILL_SWITCH_MONITOR'},
                        remapping={'thruster_status':'thruster_status_sm',
                                    'kill_switch_engage_time':'kill_switch_engage_time_sm'})

kill_switch_sm.execute()
