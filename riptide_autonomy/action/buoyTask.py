#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import LinearCommand, AlignmentCommand
import riptide_autonomy.msg

from actionWrapper import *

class BuoyTaskAction(object):

    threeBuoySides = ["Garlic", "Wolf"]

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self.alignPub = rospy.Publisher("/command/alignment", AlignmentCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "buoy_task", riptide_autonomy.msg.BuoyTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()


    def execute_cb(self, goal):
        rospy.loginfo("Starting Buoy task")
        alignAction("Cutie", .3, True).wait_for_result()
        distance = getResult(getDistanceAction("Cutie")).distance
        self.alignPub.publish("",0)
        moveAction(distance, 0).wait_for_result()
        self.xPub.publish(20, LinearCommand.FORCE)
        rospy.sleep(4)
        self.xPub.publish(0, LinearCommand.FORCE)

        moveAction(-2, 0).wait_for_result()
        self.yPub.publish(20, LinearCommand.FORCE)
        frontFace = next(x for x in self.threeBuoySides if not x == goal.backside)
        waitAction(frontFace, 5).wait_for_result()
        self.yPub.publish(0, LinearCommand.FORCE)
        distance = getResult(getDistanceAction(frontFace)).distance + 1
        arcAction(-170, -10, distance).wait_for_result()

        alignAction(goal.backside, .3).wait_for_result()
        distance = getResult(getDistanceAction(goal.backside)).distance
        moveAction(distance, 0).wait_for_result()
        self.xPub.publish(20, LinearCommand.FORCE)
        rospy.sleep(4)
        self.xPub.publish(0, LinearCommand.FORCE)

        rospy.loginfo("Finished Buoy task")

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('buoy_task')
    server = BuoyTaskAction()
    rospy.spin()
