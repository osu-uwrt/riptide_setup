#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import LinearCommand
from std_msgs.msg import String, Float64, Int8, Int32
import riptide_controllers.msg
import riptide_autonomy.msg
from actionTools import *

import time

class GarlicTaskAction(object):

    def __init__(self):
        self.dropperPub = rospy.Publisher("/command/drop", Int8, queue_size=1)
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "garlic_task", riptide_autonomy.msg.GarlicTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("Starting garlic drop")
        
        rospy.loginfo("Found object Bin")
        alignAction("Bin", 0.2).wait_for_result()
        moveAction(-.4, 0).wait_for_result()

        pitchAction(-70).wait_for_result()
        self.xPub.publish(30, LinearCommand.FORCE)
        rospy.sleep(0.4)
        self.xPub.publish(-30, LinearCommand.FORCE)
        rospy.sleep(0.4)
        self.xPub.publish(0, LinearCommand.FORCE)
        rospy.sleep(2)
        pitchAction(0).wait_for_result()

        #moveAction(0, -3).wait_for_result()
        #depthAction(2.8).wait_for_result()

        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('garlic_task')
    server = GarlicTaskAction()
    rospy.spin()
