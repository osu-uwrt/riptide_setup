#! /usr/bin/env python
import rospy
import actionlib
import dynamic_reconfigure.client

from std_msgs.msg import Float32, Float64
import riptide_autonomy.msg
import riptide_controllers.msg
from riptide_msgs.msg import SwitchState


import time
import math
from multiprocessing import Process



class BeAutonomousAction(object):

    def __init__(self):
        self.depthPub = rospy.Publisher(
            "/command/depth", DepthCommand, queue_size=5)
        self.rollPub = rospy.Publisher(
            "/command/roll", AttitudeCommand, queue_size=5)
        self.pitchPub = rospy.Publisher(
            "/command/pitch", AttitudeCommand, queue_size=5)
        self.yawPub = rospy.Publisher(
            "/command/yaw", AttitudeCommand, queue_size=5)
        self._as = actionlib.SimpleActionServer(
            "beAutonomous", riptide_autonomy.msg.beAutonomousAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
    
    def performActions(self, *actions):
        for a in actions:
            a.wait_for_result()

    def yawAction(self, angle):
        client = actionlib.SimpleActionClient(
            "go_to_yaw", riptide_controllers.msg.GoToYawAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToYawGoal(angle))
        return client

    def depthAction(self, depth):
        client = actionlib.SimpleActionClient(
            "go_to_depth", riptide_controllers.msg.GoToDepthAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToDepthGoal(depth))
        return client

    def rollAction(self, angle):
        client = actionlib.SimpleActionClient(
            "go_to_roll", riptide_controllers.msg.GoToRollAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToRollGoal(angle))
        return client

    def pitchAction(self, angle):
        client = actionlib.SimpleActionClient(
            "go_to_pitch", riptide_controllers.msg.GoToPitchAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToPitchGoal(angle))
        return client

    def GateTask(self):
        client = actionlib.SimpleActionClient(
            "gate_task", riptide_autonomy.msg.GateTask.action)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(ritpide_autonomy.msg.GateTask())
        return client
    def BuoyTask(self):
        client = actionlib.SimpleActionClient(
            "buoy_task", riptide_autonomy.msg.BuoyTask.action)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(ritpide_autonomy.msg.BuoyTask())
        return client

    def execute_cb(self, goal):

        rospy.loginfo("Waiting for killswitch")
        while not rospy.wait_for_message("/state/switches", SwitchState).kill:
            rospy.sleep(0.2)

        rospy.sleep(5)

        rospy.loginfo("Going down")

        self.performActions(
            self.depthAction(2.0),
            self.rollAction(0),
            self.pitchAction(0)
        )

        MoveToTask("Start","GateTask")
        GateTask()
        MoveToTask("GateEnd","Buoys")
        BuoyTask()


       


if __name__ == '__main__':
    rospy.init_node('beAutonomous')
    server = BeAutonomousAction()
    rospy.spin()
