#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import LinearCommand, AttitudeCommand, DepthCommand
import riptide_autonomy.msg

from actionWrapper import *

class garlicTask(object):

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.rollPub = rospy.Publisher("/command/roll", AttitudeCommand, queue_size=1)
        self.pitchPub = rospy.Publisher("/command/pitch", AttitudeCommand, queue_size=1)
        self.yawPub = rospy.Publisher("/command/yaw", AttitudeCommand, queue_size=1)
        self.depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=1)

        self._as = actionlib.SimpleActionServer(
            "go_to_finals", riptide_autonomy.msg.GoToFinalsAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def goToTask(self, task):
        yawAction(12).wait_for_result()
        self.xPub.publish(20, LinearCommand.FORCE)


    def execute_cb(self, goal):
        performActions(
            depthAction(2),
            rollAction(0),
            pitchAction(0),
            yawAction(170)
        )
        self.xPub.publish(30, LinearCommand.FORCE)
        waitAction("Cutie", 10).wait_for_result()
        buoyTaskAction("Batman").wait_for_result()

        self.yawPub.publish(0, AttitudeCommand.MOMENT)
        performActions(
            depthAction(0),
            rollAction(0),
            pitchAction(0)
        )
        self.rollPub.publish(0, AttitudeCommand.MOMENT)
        self.pitchPub.publish(0, AttitudeCommand.MOMENT)
        self.depthPub.publish(False, 0)
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('garklic_task')
    server = GarlicTaskAction()
    rospy.spin()
