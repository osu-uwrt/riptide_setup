#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import AttitudeCommand, Imu, LinearCommand
from std_msgs.msg import Float32
import riptide_controllers.msg

import time

class Navigate(object):

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yawPub = rospy.Publisher("/command/yaw", AttitudeCommand, queue_size=5)

        self._as = actionlib.SimpleActionServer(
            "navigate", riptide_controllers.msg.NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def performActions(self, *actions):
        for a in actions:
            a.wait_for_result()

    def yawAction(self, angle):
        client = actionlib.SimpleActionClient("go_to_yaw", riptide_controllers.msg.GoToYawAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToYawGoal(angle))
        return client
    
    def pitchAction(self, angle):
        client = actionlib.SimpleActionClient("go_to_pitch", riptide_controllers.msg.GoToPitchAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToPitchGoal(angle))
        return client

    def rollAction(self, angle):
        client = actionlib.SimpleActionClient("go_to_roll", riptide_controllers.msg.GoToRollAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToRollGoal(angle))
        return client
    
    def execute_cb(self, goal):
        self.xPub.publish(20, LinearCommand.FORCE)
        performActions(
            depthAction(2),
            rollAction(0),
            pitchAction(0),
            yawAction(170)
        )
        self.xPub.publish(30, LinearCommand.FORCE)
        waitAction("Cutie", 10).wait_for_result()
        buoyTaskAction("Batman").wait_for_result()

        self.yawPub.publish(0, AttitudeCommand.MOMENT)
        performActions(
            depthAction(0),
            rollAction(0),
            pitchAction(0)
        )
        self.rollPub.publish(0, AttitudeCommand.MOMENT)
        self.pitchPub.publish(0, AttitudeCommand.MOMENT)
        self.depthPub.publish(False, 0)
        self._as.set_succeeded()
