#! /usr/bin/env python
import rospy
import actionlib

import riptide_autonomy.msg
from riptide_msgs.msg import ResetControls
from std_msgs.msg import Int8

from actionTools import *
import math


def addAngle(a, b):
    return ((a+b+180) % 360) - 180


class Task:
    def __init__(self, x, y, camera, obj, action):
        self.x = x
        self.y = y
        self.camera = camera
        self.obj = obj
        self.action = action


tasks = [
    Task(17, 10, 0, "Gate", lambda: gateTaskAction(True).wait_for_result()),
    Task(31, 20, 0, "Cutie", lambda: buoyTaskAction(True, "Fairy").wait_for_result()),
    Task(22, 19, 0, "Decap", lambda: decapTaskAction().wait_for_result()),
    Task(45, 97, 1, "Bat", lambda: garlicTaskAction().wait_for_result()),
    Task(45, 97, 0, "Structure", lambda: exposeTaskAction().wait_for_result())
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
        dX = task.x - self.x
        dY = task.y - self.y
        if quadrant == 0:
            angle = addAngle(-math.atan2(dY, dX) * 180 /
                             math.pi, self.transdecOrientation + 90)
        if quadrant == 1:
            angle = addAngle(-math.atan2(dX, dY) * 180 / math.pi,
                             self.transdecOrientation + 180)
        if quadrant == 2:
            angle = addAngle(-math.atan2(dX, dY) * 180 /
                             math.pi, self.transdecOrientation)
        if quadrant == 3:
            angle = addAngle(-math.atan2(dY, dX) * 180 /
                             math.pi, self.transdecOrientation - 90)
        yawAction(angle).wait_for_result()
        searchAction(task.obj).wait_for_result()

    def execute_cb(self, goal):
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
            self.x = task.x
            self.y = task.y

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
