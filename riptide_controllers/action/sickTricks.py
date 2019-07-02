#! /usr/bin/env python
import rospy
import actionlib
import dynamic_reconfigure.client

from riptide_msgs.msg import DepthCommand, AttitudeCommand, LinearCommand, Imu, Object, Depth
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Float64, Int32
import riptide_controllers.msg

import time 
import math

class PlaySickTricks(object):
    
    def __init__(self):
        self.depthPub = rospy.Publisher(
            "/command/depth", DepthCommand, queue_size=5)
        self.rollPub = rospy.Publisher(
            "/command/roll", AttitudeCommand, queue_size=5)
        self.pitchPub = rospy.Publisher(
            "/command/pitch", AttitudeCommand, queue_size=5)
        self.yawPub = rospy.Publisher(
            "/command/yaw", AttitudeCommand, queue_size=5)
        self.XPub = rospy.Publisher(
            "/command/x", LinearCommand, queue_size=5)
        self.YPub = rospy.Publisher(
            "/command/y", LinearCommand, queue_size=5)
        self.ZPub = rospy.Publisher(
            "/command/force_z", Float64, queue_size=5)

        self._as = actionlib.SimpleActionServer(
            "sick_tricks", riptide_controllers.msg.SickTricksAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def performActions(self, *actions):
        for a in actions:
            a.wait_for_result()

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

    def yawAction(self, angle):
        client = actionlib.SimpleActionClient(
            "go_to_yaw", riptide_controllers.msg.GoToYawAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToYawGoal(angle))
        return client

    def execute_cb(self, goal):

        rospy.loginfo("Going down")
        # First let the robot go inside water
        self.performActions(
            self.depthAction(1.0),
            self.rollAction(0),
            self.pitchAction(0),
            self.yawAction(180)
        )

        if goal.maneuvers == 1:
            rospy.loginfo("Driving forward while spining in x direction")
            self.XPub.publish(20, LinearCommand.FORCE)
            self.rollPub.publish(50, AttitudeCommand.VELOCITY)
            rospy.sleep(5)
        elif goal.maneuvers == 2:
            rospy.loginfo("Driving forward")
            self.XPub.publish(0.5, LinearCommand.VELOCITY)
            rospy.sleep(3)
            rospy.loginfo("Turning right 90 degrees")
            self.performActions(
                self.yawAction(-90)
            )
            rospy.loginfo("Driving forward")
            self.XPub.publish(0.5, LinearCommand.VELOCITY)
            rospy.sleep(3)
            rospy.loginfo("Staying still two seconds")
            self.XPub.publish(0, LinearCommand.VELOCITY)
            rospy.sleep(2)
            rospy.loginfo("Driving backward")
            self.XPub.publish(-0.5, LinearCommand.VELOCITY)
            rospy.sleep(3)
            rospy.loginfo("Turning right -90 degrees")
            self.performActions(
                self.yawAction(180)
            )
            rospy.loginfo("Driving backward")
            self.XPub.publish(-0.5, LinearCommand.VELOCITY)
            rospy.sleep(3)
        elif goal.maneuvers == 3:
            rospy.loginfo("Going up and down while spining in z direction")
            self.yawPub.publish(15, AttitudeCommand.VELOCITY)
            self.performActions(self.depthAction(0.5))
            self.performActions(self.depthAction(1.0))
        elif goal.maneuvers == 4:
            rospy.loginfo("Driving sideways")
            self.performActions(self.rollAction(90))
            self.performActions(self.depthAction(.5))
            self.performActions(self.depthAction(1.0))
            self.performActions(self.rollAction(-90))
            self.ZPub.publish(50)
            rospy.sleep(4)
            self.ZPub.publish(-50)
            rospy.sleep(4)
        elif goal.maneuvers == 5:
            rospy.loginfo("Driving upside down")
            self.performActions(
                self.rollAction(180),
                self.yawAction(0)
            )
            self.performActions(self.pitchAction(45))
            self.performActions(self.pitchAction(-45))
            self.performActions(
                self.rollAction(0),
                self.yawAction(180),
                self.pitchAction(0)
            )
        elif goal.maneuvers == 6:
            rospy.loginfo("Front flip")
            self.pitchPub.publish(-90, AttitudeCommand.VELOCITY)
            rospy.sleep(4)
                    

        # Surface
        self.XPub.publish(0, LinearCommand.FORCE)
        self.YPub.publish(0, LinearCommand.FORCE)
        self.yawPub.publish(0, AttitudeCommand.MOMENT)
        self.performActions(
            self.depthAction(0.0),
            self.rollAction(0),
            self.pitchAction(0)
        )
        self.depthPub.publish(False, 0)
        self.rollPub.publish(0, AttitudeCommand.MOMENT)
        self.pitchPub.publish(0, AttitudeCommand.MOMENT)

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('sick_tricks')
    server = PlaySickTricks()
    rospy.spin()
