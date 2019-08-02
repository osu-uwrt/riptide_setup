#! /usr/bin/env python
import rospy
import actionlib

import riptide_controllers.msg
import riptide_autonomy.msg
from riptide_msgs.msg import Imu, LinearCommand, AttitudeCommand
from std_msgs.msg import Int8

from actionTools import *
import math
import time

def angleDiff(a, b):
    return ((a-b+180) % 360)-180

class ExposeTaskAction(object):
    start_ang = 0
    drive_force = 15
    
    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self.rollPub = rospy.Publisher("/command/roll", AttitudeCommand, queue_size=5)
        self.cameraPub = rospy.PUblisher("/command/camera", Int8, queue_size=1)

        self._as = actionlib.SimpleActionServer(
            "expose_task", riptide_autonomy.msg.ExposeTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.timer = rospy.Timer(rospy.Duration(0.05), lambda _: checkPreempted(self._as))

    def imuCb(self, msg):
        self.position = 20 * math.sin(math.pi / 3 * (time.time() - self.startTime))
        self.rollPub.publish(angleDiff(self.position, 0), AttitudeCommand.POSITION)

        sy = math.sin(angleDiff(msg.rpy_deg.z, self.start_ang) * math.pi / 180)
        cy = math.cos(angleDiff(msg.rpy_deg.z, self.start_ang) * math.pi / 180)

        self.xPub.publish(self.drive_force * cy, LinearCommand.FORCE)
        self.yPub.publish(self.drive_force * sy, LinearCommand.FORCE)

    def execute_cb(self, goal):
        cameraPub.publish(1)
        rospy.loginfo("Start exposing to sunlight task")
        depthAction(0.5).wait_for_result()
        self.start_ang = rospy.wait_for_message("/state/imu", Imu).rpy_deg.z
        self.startTime = time.time()

        rospy.loginfo("Looking for pinger")
        imuSub = rospy.Subscriber("/state/imu", Imu, self.imuCb)
        waitAction("Pinger", 5).wait_for_result()
        
        rospy.logingo("Found pinger, align to it")
        imuSub.unregister()
        alignAction("Pinger", 0.2).wait_for_result()

        rospy.loginfo("Go to the surface")
        self.xPub.publish(0, LinearCommand.FORCE)
        self.yPub.publish(0, LinearCommand.FORCE)
        depthAction(0)
        self._as.set_succeeded()
    
if __name__ == '__main__':
    rospy.init_node('expose_task')
    server = ExposeTaskAction()
    rospy.spin()

