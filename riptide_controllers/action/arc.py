#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import AttitudeCommand, LinearCommand, Imu
from nortek_dvl.msg import Dvl
import riptide_controllers.msg
import math


def angleDiff(a, b):
    return (a - b + 180) % 360 - 180

class Arc(object):
    P = 1

    def __init__(self):
        self.yawPub = rospy.Publisher(
            "/command/yaw", AttitudeCommand, queue_size=5)
        self.YPub = rospy.Publisher(
            "/command/y", LinearCommand, queue_size=5)

        self._as = actionlib.SimpleActionServer(
            "arc", riptide_controllers.msg.ArcAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("Driving in %fm arc"%goal.radius)
        self.lastVel = 0
        self.linearPos = 0
        self.angleTraveled = 0
        self.radius = goal.radius
        self.linearVelocity = -math.pi * goal.velocity / 180 * goal.radius
        self.startAngle = rospy.wait_for_message("/state/imu", Imu).rpy_deg.z

        self.yawPub.publish(goal.velocity, AttitudeCommand.VELOCITY)
        self.YPub.publish(self.linearVelocity, LinearCommand.VELOCITY)

        imuSub = rospy.Subscriber("/state/imu", Imu, self.imuCb)
        dvlSub = rospy.Subscriber("/state/dvl", Dvl, self.dvlCb)

        while (self.angleTraveled < goal.angle and goal.velocity > 0) or (self.angleTraveled > goal.angle and goal.velocity < 0):
            rospy.sleep(0.1)

        imuSub.unregister()
        dvlSub.unregister()
        self.yawPub.publish(0, AttitudeCommand.VELOCITY)
        self.YPub.publish(0, LinearCommand.VELOCITY)

        self._as.set_succeeded()

    def imuCb(self, msg):
        self.angleTraveled = angleDiff(msg.rpy_deg.z, self.startAngle)

    def dvlCb(self, msg):
        if not math.isnan(msg.velocity.x):
            curVel = msg.velocity.y
        else:
            curVel = self.lastVel
        self.linearPos += (self.lastVel + curVel) / 2 / 8 # / 8 because this message comes in at 8 Hz
        self.lastVel = curVel
        targetPos = -math.pi * self.angleTraveled / 180 * self.radius

        self.YPub.publish(self.linearVelocity + self.P * (targetPos - self.linearPos), LinearCommand.VELOCITY)


if __name__ == '__main__':
    rospy.init_node('arc')
    server = Arc()
    rospy.spin()
