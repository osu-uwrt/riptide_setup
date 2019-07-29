#! /usr/bin/env python
import rospy
import actionlib
import dynamic_reconfigure.client

from riptide_msgs.msg import AttitudeCommand, LinearCommand, Imu
from std_msgs.msg import Float32, Float64, Int32
import riptide_controllers.msg

import time
import math
import numpy as np


def angleDiff(a, b):
    return ((a-b+180) % 360)-180


class GateManeuver(object):

    ROLL_P = 2
    CRUISE_VELOCITY = 45
    DRIVE_FORCE = 30

    def __init__(self):
        self.rollPub = rospy.Publisher(
            "/command/roll", AttitudeCommand, queue_size=5)
        self.yawPub = rospy.Publisher(
            "/command/yaw", AttitudeCommand, queue_size=5)
        self.XPub = rospy.Publisher(
            "/command/x", LinearCommand, queue_size=5)
        self.YPub = rospy.Publisher(
            "/command/y", LinearCommand, queue_size=5)
        self.ZPub = rospy.Publisher(
            "/command/force_z", Float64, queue_size=5)

        self._as = actionlib.SimpleActionServer(
            "gate_maneuver", riptide_controllers.msg.GateManeuverAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("Starting gate maneuver")
        self.startAngle = rospy.wait_for_message("/state/imu", Imu).rpy_deg.z
        self.angleTraveled = 0
        self.pastHalf = False

        self.yawPub.publish(self.CRUISE_VELOCITY, AttitudeCommand.VELOCITY)
        self.rollPub.publish(self.CRUISE_VELOCITY, AttitudeCommand.VELOCITY)

        self.imuSub = rospy.Subscriber("/state/imu", Imu, self.imuCb)

        while self.angleTraveled < 330 and not rospy.is_shutdown():
            rospy.sleep(0.05)

            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Gate Maneuver')
                self.cleanup()
                self._as.set_preempted()
                return

        rospy.loginfo("Leveling")

        self.cleanup()

        while abs(rospy.wait_for_message("/state/imu", Imu).rpy_deg.x) > 5 and not rospy.is_shutdown():
            rospy.sleep(0.05)

        rospy.loginfo("Done")

        self._as.set_succeeded()

    def cleanup(self):
        self.yawPub.publish(0, AttitudeCommand.POSITION)
        self.rollPub.publish(0, AttitudeCommand.POSITION)
        self.imuSub.unregister()
        self.XPub.publish(0, LinearCommand.FORCE)
        self.YPub.publish(0, LinearCommand.FORCE)
        self.ZPub.publish(0)

    def imuCb(self, msg):
        self.angleTraveled = angleDiff(msg.rpy_deg.z, self.startAngle)
        roll = msg.rpy_deg.x
        if self.angleTraveled < -90:
            self.pastHalf = True
        if self.pastHalf and self.angleTraveled < 0:
            self.angleTraveled += 360
        if roll < 0:
            roll += 360

        self.rollPub.publish(self.CRUISE_VELOCITY + self.ROLL_P * (self.angleTraveled - roll), AttitudeCommand.VELOCITY)

        sr = math.sin(roll * math.pi / 180)
        cr = math.cos(roll * math.pi / 180)
        sy = math.sin(self.angleTraveled * math.pi / 180)
        cy = math.cos(self.angleTraveled * math.pi / 180)

        rRotMat = np.matrix([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        yRotMat = np.matrix([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        outVector = np.dot(np.linalg.inv(np.dot(yRotMat, rRotMat)), np.matrix([[self.DRIVE_FORCE],[0],[0]]))

        self.XPub.publish(outVector.item(0), LinearCommand.FORCE)
        self.YPub.publish(outVector.item(1), LinearCommand.FORCE)
        self.ZPub.publish(outVector.item(2))


if __name__ == '__main__':
    rospy.init_node('gate_maneuver')
    server = GateManeuver()
    rospy.spin()
