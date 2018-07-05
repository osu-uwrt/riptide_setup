#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs.msg import Constants, ResetControls, SwitchState
import auv_concurrence_sm
import subprocess

class Standby(State):
    kill_switch_status = STATUS_INIT
    restart_switch_status = STATUS_INIT
    shutdown_switch_status = STATUS_INIT
    mission_switch_status = STATUS_INIT

    def __init__(self):
        State.__init__(self, outcome=['execute_mission', 'standby', 'end'],
                        input_keys=['mission_status'],
                        output_keys=[])
        rospy.init_node('idle')
        controller_pub = rospy.Publisher('/controls/reset', ResetControls, queue_size=1)
        switch_sub = rospy.Subscriber("/state/switches", SwitchState, callback)

    def execute(self, userdata):
        #Act on master_switch_status
        #Reset all controllers first, then loop back (kill disengaged)
        #or run script to restart or shutdown computer

        #Reset Controllers
        controller_msg = ResetControls()
        controller_msg.reset_surge = True
        controller_msg.reset_sway = True
        controller_msg.reset_heave = True
        controller_msg.reset_roll = True
        controller_msg.reset_pitch = True
        controller_msg.reset_yaw = True
        controller_msg.reset_pwm = True
        controller_pub.Publish(controller_msg)

        if mission_switch_status == STATUS_ACTIVATED && userdata.mission_status == STATUS_INIT
            return 'execute_mission'
        elif mission_switch_status == STATUS_DEACTIVATED && userdata.mission_status == STATUS_MISSION_ATTEMPTED
            return 'execute_mission'
        elif restart_switch_status == STATUS_ACTIVATED:
            cmd = "reboot -h"
            #process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
            rospy.loginfo('Restart computer')
            return 'end'
        elif shutdown_switch_status == STATUS_ACTIVATED:
            cmd = "shutdown -h now"
            #process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
            rospy.loginfo('Shutdown computer')
            return 'end'
        else #kill_switch_status == STATUS_DEACTIVATED:
            rospy.loginfo('Standby')
            return 'standby'

        #Otherwise, act on mission_status (wait for kill switch to be
        #disengaged and then reengaged)

    def callback(data):
        # Get kill switch status
        if data.kill == True:
            kill_switch_status = STATUS_DEACTIVATED
        else
            kill_switch_status = STATUS_ACTIVATED

        # Get mission switch status
        if data.sw1 == True && kill_switch_status = STATUS_DEACTIVATED:
            mission_switch_status = STATUS_ACTIVATED
        else
            mission_switch_status = STATUS_DEACTIVATED

        # Get computer switch statuses
        if data.sw3 == True && kill_switch_status == STATUS_ACTIVATED:
            restart_switch_status = STATUS_ACTIVATED
        elif data.sw4 == True && kill_switch_status == STATUS_ACTIVATED:
            shutdown_switch_status = STATUS_ACTIVATED
        else
            restart_switch_status = STATUS_DEACTIVATED
            shutdown_switch_status = STATUS_DEACTIVATED

auv_controller_sm = StateMachine(outcomes = ['end'],
                            input_keys=['mission_status'],
                            output_keys=['mission_status'])
auv_controller_sm.userdata.mission_status = STATUS_INIT

with auv_controller_sm:
    StateMachine.add('AUV_CONCURRENCE_SM', auv_concurrence_sm,
                    transitions={'exit':'STANDBY'},
                    remapping={'mission_status':'mission_status'})
    StateMachine.add('STANDBY', Standbye(),
                    transitions={'execute_mission':'AUV_CONCURRENCE_SM',
                                'standby':'STANDBY'},
                    remapping={'mission_status':'mission_status'})

def main()
    rospy.init_node('sentience')
    auv_controller_sm.execute()
    ros.spin()

main()
