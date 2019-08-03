#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import AlignmentCommand, LinearCommand
import riptide_autonomy.msg

from actionTools import *

class GateTaskAction(object):

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.alignPub = rospy.Publisher("/command/alignment", AlignmentCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "gate_task", riptide_autonomy.msg.GateTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.timer = rospy.Timer(rospy.Duration(0.05), lambda _: checkPreempted(self._as))


    def execute_cb(self, goal):
        rospy.loginfo("Aligning to gate")
        alignAction("Gate", .07, True).wait_for_result()
        distance = getResult(getDistanceAction("Gate")).distance
        self.alignPub.publish("", 0)
        depthAction(.65).wait_for_result()
        # Get 2.0 meters away from the gate
        if goal.isLeft:
            rospy.loginfo("Moving left")
            moveAction(distance - 2, -.8).wait_for_result()
        else:
            rospy.loginfo("Moving right")
            moveAction(distance - 2, .8).wait_for_result()

        rospy.loginfo("Stand back and watch this!")
        gateManeuverAction().wait_for_result()

        rospy.loginfo("Moving to path")
        if goal.isLeft:
            moveAction(0, 1).wait_for_result()
        else:
            rospy.loginfo("Moving right")
            moveAction(0, -1).wait_for_result()

        rospy.loginfo("Gate task completed")
        depthAction(2).wait_for_result()

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('gate_task')
    server = GateTaskAction()
    rospy.spin()
