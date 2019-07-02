#! /usr/bin/env python
import rospy
import actionlib
import dynamic_reconfigure.client

from riptide_msgs.msg import DepthCommand, AttitudeCommand, AlignmentCommand, Imu, Object, ControlStatusLinear, SwitchState, Depth
from geometry_msgs.msg import Vector3Stamped, Point
from std_msgs.msg import Float32, Float64
import riptide_autonomy.msg
import riptide_controllers.msg

import time
import math
from multiprocessing import Process


GATE_HEADING = 110


def angleDiff(a1, a2):
    return (a1 - a2 + 180) % 360 - 180


class PrequalifyAction(object):

    def __init__(self):
        self.depthPub = rospy.Publisher(
            "/command/depth", DepthCommand, queue_size=5)
        self.rollPub = rospy.Publisher(
            "/command/roll", AttitudeCommand, queue_size=5)
        self.pitchPub = rospy.Publisher(
            "/command/pitch", AttitudeCommand, queue_size=5)
        self.yawPub = rospy.Publisher(
            "/command/yaw", AttitudeCommand, queue_size=5)
        self.alignmentPub = rospy.Publisher(
            "/command/alignment", AlignmentCommand, queue_size=5)
        self.forcePub = rospy.Publisher(
            "/command/force_x", Float64, queue_size=5)

        self._as = actionlib.SimpleActionServer(
            "prequalify", riptide_autonomy.msg.PrequalifyAction, execute_cb=self.execute_cb, auto_start=False)
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

        rospy.loginfo("Waiting for killswitch")
        while not rospy.wait_for_message("/state/switches", SwitchState).kill:
            rospy.sleep(0.2)

        rospy.sleep(5)

        rospy.loginfo("Going down")

        # Submerge and face gate
        self.performActions(
            self.depthAction(2.0),
            self.rollAction(0),
            self.pitchAction(0),
            self.yawAction(GATE_HEADING)
        )

        rospy.loginfo("Headed forward")
        # Drive forward
        self.forcePub.publish(30)
        rospy.sleep(4)

        # Wait until you see the pole a few times
        count = 0
        lastTime = time.time()
        while count < 5:
            rospy.wait_for_message("/state/object", Object)
            diff = time.time() - lastTime
            if diff < 0.3:
                count += 1
            else:
                count = 0
            lastTime = time.time()

        rospy.loginfo("Aligning to pole")
        # Align to pole
        cmd = AlignmentCommand(True, True, False, "", 0, 0, 0, Point(2.5, 0, 0))
        self.alignmentPub.publish(cmd)

        # Wait until aligned
        while abs(rospy.wait_for_message("/status/controls/linear", ControlStatusLinear).x.error) > .2:
            rospy.sleep(0.05)

        rospy.loginfo("Going around")
        # Start rotating while still aligning
        self.yawPub.publish(7, AttitudeCommand.VELOCITY)

        # Wait until on other side of pole
        done = False
        while not done:
            angle = rospy.wait_for_message("/state/imu", Imu).rpy_deg.z
            diff = angleDiff(angle, GATE_HEADING)
            if diff > -100 and diff < -40:
                done = True

        rospy.loginfo("Turning back to gate")
        # Shut off alignment and turn to gate
        self.alignmentPub.publish(AlignmentCommand(False, False, False, "",
                               0, 0, 0, Point(0, 0, 0)))
        self.performActions(self.yawAction(angleDiff(GATE_HEADING, 180)))

        rospy.loginfo("Headed back to gate")
        # Drive forward for a while
        self.forcePub.publish(30)
        rospy.sleep(30)

        rospy.loginfo("Done. Easy peasy")
        # Surface
        self.forcePub.publish(0)
        self.depthPub.publish(False, 0)
        self.rollPub.publish(0, AttitudeCommand.MOMENT)
        self.pitchPub.publish(0, AttitudeCommand.MOMENT)
        self.yawPub.publish(0, AttitudeCommand.MOMENT)

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('prequalify')
    server = PrequalifyAction()
    rospy.spin()
