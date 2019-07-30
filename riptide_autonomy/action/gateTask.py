#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import AlignmentCommand
import riptide_autonomy.msg

from actionTools import *

class GateTaskAction(object):

    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            "gate_task", riptide_autonomy.msg.GateTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.timer = rospy.Timer(rospy.Duration(0.05), lambda _: checkPreempted(self._as))


    def execute_cb(self, goal):
        rospy.loginfo("Aligning to gate")
        alignAction("Gate", .15).wait_for_result()
        if goal.isLeft:
            rospy.loginfo("Moving left")
            moveAction(0, -1).wait_for_result()
        else:
            rospy.loginfo("Moving right")
            moveAction(0, 1).wait_for_result()

        rospy.loginfo("Stand back and watch this!")
        gateManeuverAction().wait_for_result()

        rospy.loginfo("Gate task completed")

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('gate_task')
    server = GateTaskAction()
    rospy.spin()
