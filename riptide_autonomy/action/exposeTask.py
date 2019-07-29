#! /usr/bin/env python
import rospy
import actionlib

import riptide_controllers.msg
import riptide_autonomy.msg
from actionTools import *

class ExposeTaskAction(object):
    
    def __init__(self):

        self._as = actionlib.SimpleActionServer(
            "expose_task", riptide_autonomy.msg.ExposeTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        depthAction(0.0).wait_for_result()
        self._as.set_succeeded()
    
if __name__ == '__main__':
    rospy.init_node('expose_task')
    server = ExposeTaskAction()
    rospy.spin()

