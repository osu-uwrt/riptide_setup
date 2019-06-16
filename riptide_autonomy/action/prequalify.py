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


GATE_HEADING = 20


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

    def setDepth(self, depth):
        # rospy.loginfo("Waiting for depth service")
        # client = actionlib.SimpleActionClient(
        #     "go_to_depth", riptide_controllers.msg.GoToDepthAction)
        # client.wait_for_server()
        # rospy.loginfo("Got depth service")

        # # Sends the goal to the action server.
        # client.send_goal(riptide_controllers.msg.GoToDepthAction(depth))
        # rospy.loginfo("Sent goal")

        # # Waits for the server to finish performing the action.
        # client.wait_for_result()
        # rospy.loginfo("At depth")
        self.depthPub.publish(True, depth)

        rospy.loginfo("Waiting on depth")
        while abs(rospy.wait_for_message("/state/depth", Depth).depth - depth) > 0.1:
            rospy.sleep(0.05)
        rospy.loginfo("At depth")

    def setHeading(self, angle):
        self.yawPub.publish(angle, AttitudeCommand.POSITION)

        while abs(angleDiff(rospy.wait_for_message("/state/imu", Imu).rpy_deg.z, angle)) >= 5:
            rospy.sleep(0.05)

    def setPitch(self, angle):
        self.pitchPub.publish(angle, AttitudeCommand.POSITION)

        while abs(angleDiff(rospy.wait_for_message("/state/imu", Imu).rpy_deg.y, angle)) >= 5:
            rospy.sleep(0.05)

    def setRoll(self, angle):
        self.rollPub.publish(angle, AttitudeCommand.POSITION)

        while abs(angleDiff(rospy.wait_for_message("/state/imu", Imu).rpy_deg.x, angle)) >= 5:
            rospy.sleep(0.05)

    def runParallel(self, procs):
        for p in procs:
            p.start()
        for p in procs:
            p.join()

    def execute_cb(self, goal):

        rospy.loginfo("Waiting for killswitch")
        while not rospy.wait_for_message("/state/switches", SwitchState).kill:
            rospy.sleep(0.2)

        rospy.sleep(5)

        rospy.loginfo("Going down")
        # Submerge and face gate
        self.setDepth(1) 
        self.setHeading(GATE_HEADING)
        self.setPitch(0)
        self.setRoll(0)

        rospy.loginfo("Headed forward")
        # Drive forward
        self.forcePub.publish(40)

        # Wait until you see the pole a few times
        count = 0
        lastTime = time.time()
        while count < 3:
            rospy.wait_for_message("/state/object", Object)
            diff = time.time() - lastTime
            if diff < 0.5:
                count += 1
            lastTime = time.time()

        rospy.loginfo("Aligning to pole")
        # Align to pole
        cmd = AlignmentCommand(True, True, False, "", 0, 0, 0, Point(2, 0, 0))
        self.alignmentPub.publish(cmd)

        # Wait until aligned
        while abs(rospy.wait_for_message("/status/controls/linear", ControlStatusLinear).x.error) > .2:
            rospy.sleep(0.05)

        rospy.loginfo("Going around")
        # Start rotating while still aligning
        self.yawPub.publish(90, AttitudeCommand.VELOCITY)

        # Wait until on other side of pole
        done = False
        while not done:
            angle = rospy.wait_for_message("/state/imu", Imu).rpy_deg.z
            diff = angleDiff(angle, GATE_HEADING)
            if diff > -90 and diff < -20:
                done = True

        rospy.loginfo("Turning back to gate")
        # Shut off alignment and turn to gate
        self.alignmentPub.publish(AlignmentCommand(False, False, False, "",
                               0, 0, 0, Point(0, 0, 0)))
        self.setHeading(angleDiff(GATE_HEADING, 170))

        rospy.loginfo("Headed back to gate")
        # Drive forward for a while
        self.forcePub.publish(40)
        rospy.sleep(5)

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
