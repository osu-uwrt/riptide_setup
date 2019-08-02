#! /usr/bin/env python
import rospy
import actionlib

import riptide_autonomy.msg
from riptide_msgs.msg import ResetControls, SwitchState
from std_msgs.msg import Int8, Float32

from actionTools import *
import math


def addAngle(a, b):
    return ((a+b+180) % 360) - 180


class Task:
    def __init__(self, heading, camera, obj, action):
        self.heading = heading
        self.camera = camera
        self.obj = obj
        self.action = action


tasks = [
    Task(15.0, 0, "Gate", lambda: gateTaskAction(True).wait_for_result()),
    Task(15.0, 0, "Cutie", lambda: buoyTaskAction(True, "Fairy").wait_for_result()),
    Task(80.0, 0, "Decap", lambda: decapTaskAction().wait_for_result()),
    Task(-40.0, 1, "Bat", lambda: garlicTaskAction().wait_for_result()),
    Task(30.0, 0, "Structure", lambda: exposeTaskAction().wait_for_result())
]


class GoToFinalsAction(object):
    transdecOrientation = -5

    def __init__(self):
        self.resetPub = rospy.Publisher(
            "/controls/reset", ResetControls, queue_size=1)
        self.camPub = rospy.Publisher(
            "/command/camera", Int8, queue_size=1)

        self._as = actionlib.SimpleActionServer(
            "go_to_finals", riptide_autonomy.msg.GoToFinalsAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.timer = rospy.Timer(rospy.Duration(
            0.05), lambda _: checkPreempted(self._as))

    def goToTask(self, task, quadrant):

        if quadrant == 0:
            angle = addAngle(-task.heading, self.transdecOrientation + 90)
        if quadrant == 1:
            angle = addAngle(task.heading, self.transdecOrientation + 90)
        if quadrant == 2:
            angle = addAngle(task.heading, self.transdecOrientation - 90)
        if quadrant == 3:
            angle = addAngle(-task.heading, self.transdecOrientation - 90)
        yawAction(angle).wait_for_result()
        searchAction(task.obj).wait_for_result()

    def execute_cb(self, goal):
        rospy.loginfo("Wait for kill switch")

        while not rospy.wait_for_message("/state/switches", SwitchState).kill:
            rospy.sleep(0.1)

        rospy.sleep(5.0)
        rospy.loginfo("Kill switch plugged in, start searching for gate")

        self.resetPub.publish(False)
        performActions(
            depthAction(2),
            rollAction(0),
            pitchAction(0)
        )
        self.x = 0
        self.y = 0

        for task in tasks:
            self.camPub.publish(task.camera)
            self.goToTask(task, goal.quadrant)
            task.action()

        performActions(
            depthAction(0),
            rollAction(0),
            pitchAction(0)
        )
        self.resetPub.publish(True)

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('gate_task')
    server = GoToFinalsAction()
    rospy.spin()
