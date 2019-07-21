#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import LinearCommand
import riptide_autonomy.msg

from actionWrapper import *

class BuoyTaskAction(object):

    threeBuoySides = ["Garlic", "Wolf"]

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "buoy_task", riptide_autonomy.msg.BuoyTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()


    def execute_cb(self, goal):
        rospy.loginfo("Starting Buoy task")
        alignAction("Cutie", .3).wait_for_result()
        distance = getResult(getDistanceAction("Cutie")).distance
        moveAction(distance, 0).wait_for_result()
        self.xPub.publish(10, LinearCommand.FORCE)
        rospy.sleep(2)
        self.xPub.publish(0, LinearCommand.FORCE)

        rospy.loginfo("Going around")
        moveAction(-1.5, 0).wait_for_result()
        distance = getResult(getDistanceAction("Cutie")).distance
        arcAction(-170, -10, distance).wait_for_result()

        alignAction(goal.backside, .3).wait_for_result()
        distance = getResult(getDistanceAction(goal.backside)).distance
        moveAction(distance, 0).wait_for_result()
        self.xPub.publish(10, LinearCommand.FORCE)
        rospy.sleep(2)
        self.xPub.publish(0, LinearCommand.FORCE)
        moveAction(-2, 0).wait_for_result()

        rospy.loginfo("Finished Buoy task")

        # moveAction(-3, 0).wait_for_result()
        # self.yPub.publish(20, LinearCommand.FORCE)
        # frontFace = next(x for x in self.threeBuoySides if not x == goal.backside)
        # waitAction(frontFace, 5).wait_for_result()
        # distance = getResult(getDistanceAction(frontFace)).distance
        # arcAction(-170, -10, distance).wait_for_result()

        # alignAction(goal.backside, .4).wait_for_result()
        # distance = getResult(getDistanceAction(goal.backside)).distance
        # moveAction(distance, 0).wait_for_result()
        # self.xPub.publish(10, LinearCommand.FORCE)
        # rospy.sleep(2)
        # self.xPub.publish(0, LinearCommand.FORCE)

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('buoy_task')
    server = BuoyTaskAction()
    rospy.spin()
