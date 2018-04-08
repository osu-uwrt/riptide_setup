#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs.msg import Constants, SwitchState

class MasterSwitchMonitor(State):
    kill_switch_status = 0
    restart_switch_status = 0
    shutdown_switch_status = 0

    def __init__(self):
        State.__init__(self, outcome=['master_switch_activated','master_switch_deactivated'],
                        input_keys=[],
                        output_keys=['master_switch_status'])
        rospy.init_node('master_switch_monitor')
        copro_sub = rospy.Subscriber("/state/switches", SwitchState, callback)

    def execute(self, userdata):
        #Return 'master_switch_activated' if one of the following switches
        #is activated by ORDER OF IMPORTANCE:

        if restart_switch_status == STATUS_ACTIVATED:
            userdata.master_switch_status = MASTER_SWITCH_RESTART
            return 'master_switch_activated'
        elif shutdown_switch_status == STATUS_ACTIVATED:
            userdata.master_switch_status = MASTER_SWITCH_SHUTDOWN
            return 'master_switch_activated'
        elif kill_switch_status == STATUS_ACTIVATED:
            userdata.master_switch_status = MASTER_SWITCH_KILL
            return 'master_switch_activated'
        else:
            return 'master_switch_deactivated'

    def callback(data):
        # Get kill switch status
        if data.kill == True:
            kill_switch_status = STATUS_DEACTIVATED
        elif data.kill == False:
            kill_switch_status = STATUS_ACTIVATED

        #Get computer restart switch status
        if data.sw4 == True:
            restart_switch_status = STATUS_ACTIVATED
        elif data.sw4 == False:
            restart_switch_status = STATUS_DEACTIVATED

        #Get computer shutdown switch status
        if data.sw5 == True:
            shutdown_switch_status = STATUS_ACTIVATED
        elif data.sw5 == False:
            shutdown_switch_status = STATUS_DEACTIVATED

master_switch_monitor_sm = StateMachine(outcomes=['master_switch_activated'],
                                        input_keys=[],
                                        output_keys=['master_switch_status'])
master_switch_monitor_sm.userdata.master_switch_status = 0

with master_switch_monitor_sm:
    StateMachine.add('MASTER_SWITCH_MONITOR', MasterSwitchMonitor(),
                        transitions={'master_switch_activated':'master_switch_activated',
                                    'master_switch_deactivated':'MASTER_SWITCH_MONITOR'},
                        remapping={'master_switch_status':'master_switch_status'})

outcome = master_switch_monitor_sm.execute()
