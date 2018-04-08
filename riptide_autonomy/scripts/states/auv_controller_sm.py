#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs.msg import Constants, ResetControls
import auv_concurrence_sm
import subprocess

class Idle(State):
    kill_switch_status = 0

    def __init__(self):
        State.__init__(self, outcome=['loop'],
                        input_keys=['master_switch_status', 'mission_status'],
                        output_keys=[])
        rospy.init_node('kill_switch_monitor')
        copro_sub = rospy.Subscriber("/state/switches", SwitchState, callback)

    def execute(self, userdata):
        #Act on master_switch_status
        #Reset all controllers first, then loop back (kill disengaged)
        #or run script to restart or shutdown computer

        if master_switch_status == MASTER_SWITCH_RESTART:
            cmdCommand = "reboot -h"
            process = subprocess.Popen(cmdCommand.split(), stdout=subprocess.PIPE)
        elif master_switch_status == MASTER_SWITCH_SHUTDOWN:
            cmdCommand = "shutdown -h now"
            process = subprocess.Popen(cmdCommand.split(), stdout=subprocess.PIPE)
        elif master_switch_status == MASTER_SWITCH_KILL:
            return 'loop'

        #Otherwise, act on mission_status (wait for kill switch to be
        #disengaged and then reengaged)

    def callback(data):
        # Get kill switch status
        if data.kill == True:
            kill_switch_status = STATUS_DEACTIVATED
        elif data.kill == False:
            kill_switch_status = STATUS_ACTIVATED

    def pubRestartControllerMsg():


auv_controller_sm = StateMachine(outcomes = ['loop'],
                            input_keys=['master_switch_status', 'mission_status'],
                            output_keys=[])
auv_controller_sm.userdata.master_switch_status = 0
auv_controller_sm.userdata.mission_status = 0

with auv_controller_sm:
    StateMachine.add('AUV_CONCURRENCE_SM', auv_concurrence_sm,
                    transitions={},
                    remapping={'master_switch_status':'master_switch_status',
                                'mission_status':'mission_status'})
    StateMachine.add('IDLE', Idle(),
                    transitions={'loop':'AUV_CONCURRENCE_SM',
                                'limbo':'LIMBO'},
                    remapping={'master_switch_status':'master_switch_status',
                                'mission_status':'mission_status'})

auv_controller_sm.execute()
