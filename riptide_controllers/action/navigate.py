#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import AttitudeCommand, Imu, LinearCommand
from std_msgs.msg import Float32
import riptide_controllers.msg

import time
import math
import numpy as np

def angleDiff(a, b):
    return ((a-b+180) % 360)-180

class Navigate(object):

    startTime = time.time()
    drive_force = 40
    start_ang = 0.0

    def waitAction(self, obj, times):
        client = actionlib.SimpleActionClient("wait", riptide_controllers.msg.WaitAction)
        client.wait_for_server()

        client.send_goal(riptide_controllers.msg.WaitGoal(obj, times))
        return client

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self.yawPub = rospy.Publisher("/command/yaw", AttitudeCommand, queue_size=5)

        self._as = actionlib.SimpleActionServer(
            "navigate", riptide_controllers.msg.NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
    
    def execute_cb(self, goal):
        self.yawPub.publish(goal.drive_ang, AttitudeCommand.POSITION)
        rospy.loginfo("Start navigating %s", goal.obj)
        self.start_ang = goal.drive_ang

        imuSub = rospy.Subscriber("/state/imu", Imu, self.imuCb)
        self.waitAction(goal.obj, 5).wait_for_result()

        self._as.set_succeeded()

    def imuCb(self, msg):
        self.position = 20 * math.sin(math.pi / 3 * (time.time() - sef.startTime)) + self.start_ang
        self.yawPub.publish(angleDiff(self.position, 0), AttitudeCommand.POSITION)

        sy = math.sin(angleDiff(msg.rpy_deg.z, self.start_ang) * math.pi / 180)
        cy = math.cos(angleDiff(msg.rpy_deg.z, self.start_ang) * math.pi / 180)

        self.xPub.publish(self.drive_force * cy, LinearCommand.FORCE)
        self.yPub.publish(self.drive_force * sy, LinearCommand.FORCE)

if __name__ == '__main__':
    rospy.init_node('navigate')
    server = Navigate()
    rospy.spin()