#! /usr/bin/env python
import rospy
import actionlib

import riptide_controllers.msg
import riptide_autonomy.msg
from riptide_msgs.msg import Imu, LinearCommand, AttitudeCommand
from std_msgs.msg import Int8

from actionTools import *

class ExposeTaskAction(object):
    start_ang = 0
    drive_force = 15
    
    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self.rollPub = rospy.Publisher("/command/roll", AttitudeCommand, queue_size=5)
        self.cameraPub = rospy.Publisher("/command/camera", Int8, queue_size=1)

        self._as = actionlib.SimpleActionServer(
            "expose_task", riptide_autonomy.msg.ExposeTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.timer = rospy.Timer(rospy.Duration(0.05), lambda _: checkPreempted(self._as))


    def execute_cb(self, goal):
        rospy.loginfo("Start exposing to sunlight task")
        alignAction("Pinger", 0.2).wait_for_result()

        depthAction(1.5).wait_for_result()
        moveAction(1, 0).wait_for_result()

        self.cameraPub.publish(1)
        alignAction("Pinger", 0.001).wait_for_result()

        rospy.loginfo("Finished Expose task")
        self._as.set_succeeded()
    
if __name__ == '__main__':
    rospy.init_node('expose_task')
    server = ExposeTaskAction()
    rospy.spin()

