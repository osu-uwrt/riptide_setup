#! /usr/bin/env python
import rospy
import actionlib
import dynamic_reconfigure.client

from riptide_msgs.msg import DepthCommand, AttitudeCommand, AlignmentCommand, Imu, Object, ControlStatusLinear
from geometry_msgs.msg import Vector3Stamped, Point
from std_msgs.msg import Float32, Float64
import riptide_controllers.msg

import time
import math
import asyncio
from concurrent.futures import ProcessPoolExecutor


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
            "/command/pitch", AttitudeCommand, queue_size=5)
        self.alignmentPub = rospy.Publisher(
            "/command/alignment", AlignmentCommand, queue_size=5)
        self.forcePub = rospy.Publisher(
            "/command/force_x", Float64, queue_size=5)

        self._as = actionlib.SimpleActionServer(
            "prequalify", riptide_controllers.msg.CalibrateAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    async def setDepth(self, depth):
        client = actionlib.SimpleActionClient(
            'go_to_depth', riptide_controllers.msg.GoToDepthAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToDepthAction(depth))

        # Waits for the server to finish performing the action.
        await self.loop.run_in_executor(ProcessPoolExecutor(), client.wait_for_result)

    async def setHeading(self, angle):
        self.yawPub.publish(angle, AttitudeCommand.POSITION)

        while abs(angleDiff(rospy.wait_for_message("/state/imu", Imu).rpy_deg.z, angle)) < 5:
            await self.loop.run_in_executor(ProcessPoolExecutor(), rospy.sleep, 0.05)

    async def setPitch(self, angle):
        self.pitchPub.publish(angle, AttitudeCommand.POSITION)

        while abs(angleDiff(rospy.wait_for_message("/state/imu", Imu).rpy_deg.y, angle)) < 5:
            await self.loop.run_in_executor(ProcessPoolExecutor(), rospy.sleep, 0.05)

    async def setRoll(self, angle):
        self.rollPub.publish(angle, AttitudeCommand.POSITION)

        while abs(angleDiff(rospy.wait_for_message("/state/imu", Imu).rpy_deg.x, angle)) < 5:
            await self.loop.run_in_executor(ProcessPoolExecutor(), rospy.sleep, 0.05)

    async def prequalify(self):

        # Submerge and face gate
        await asyncio.wait(
            [self.setDepth(1), self.setHeading(GATE_HEADING), self.setPitch(0), self.setRoll(0)])

        # Drive forward
        self.forcePub.publish(40)

        # Wait until you see the pole a few times
        count = 0
        lastTime = time.time()
        while count < 3:
            rospy.wait_for_message("/state/object", Object)
            if time.time() - lastTime < 0.5:
                count += 1
            lastTime = time.time()
        
        # Align to pole
        cmd = AlignmentCommand(True, True, False, "", 0, 0, 0, Point(2, 0, 0))
        self.alignmentPub.publish(cmd)

        # Wait until aligned
        while abs(rospy.wait_for_message("/status/controls/linear", ControlStatusLinear).x) < .2:
            rospy.sleep(0.05)

        # Start rotating while still aligning
        self.yawPub.publish(90, AttitudeCommand.VELOCITY)

        # Wait until on other side of gate
        done = False
        while not done:
            angle = rospy.wait_for_message("/state/imu", Imu).rpy_deg.z
            diff = angleDiff(angle, GATE_HEADING)
            if diff > -90 or diff < -20:
                done = True

        # Shut off alignment and turn to gate
        cmd = AlignmentCommand(False, False, False, "", 0, 0, 0, Point(0, 0, 0))
        self.alignmentPub.publish(cmd)
        await self.setHeading(angleDiff(GATE_HEADING, 170))

        # Drive forward for a while
        self.forcePub.publish(40)
        rospy.sleep(5)

        # Surface
        self.forcePub.publish(0)
        self.depthPub.publish(False, 0)
        self.rollPub.publish(0, AttitudeCommand.MOMENT)
        self.pitchPub.publish(0, AttitudeCommand.MOMENT)
        self.yawPub.publish(0, AttitudeCommand.MOMENT)

        

    def execute_cb(self, goal):
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.prequalify())

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('prequalify')
    server = PrequalifyAction()
    rospy.spin()
