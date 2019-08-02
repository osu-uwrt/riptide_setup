#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import LinearCommand, AlignmentCommand
import riptide_autonomy.msg

from actionTools import *

class BuoyTaskAction(object):

    threeBuoySides = ["Garlic", "Wolf"]

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self.alignPub = rospy.Publisher("/command/alignment", AlignmentCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "buoy_task", riptide_autonomy.msg.BuoyTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.timer = rospy.Timer(rospy.Duration(0.05), lambda _: checkPreempted(self._as))


    def execute_cb(self, goal):
        rospy.loginfo("Starting Buoy task")
        alignAction("Cutie", .3, True).wait_for_result()
        distance = getResult(getDistanceAction("Cutie")).distance
        self.alignPub.publish("",0)
        moveAction(distance, -.2).wait_for_result()
        if self._as.is_preempt_requested():
            rospy.loginfo('Preempted Buoy Task')
            self._as.set_preempted()
            return
        rospy.loginfo("Tapping cutie")
        self.xPub.publish(20, LinearCommand.FORCE)
        rospy.sleep(4)
        self.xPub.publish(0, LinearCommand.FORCE)

        rospy.loginfo("Backing up")
        # Back up, negative distance
        moveAction(-distance, 0.2).wait_for_result()
        alignAction("Cutie", .3, True).wait_for_result()
        distance = getResult(getDistanceAction("Cutie")).distance
        self.alignPub.publish("",0)

        if goal.isCutieLeft:
            arcAction(170, 10, distance + .6).wait_for_result()
            self.yPub.publish(-20, LinearCommand.FORCE)
        else:
            arcAction(-170, -10, distance + .6).wait_for_result()
            self.yPub.publish(20, LinearCommand.FORCE)
        
        waitAction(goal.backside, 3).wait_for_result()
        alignAction(goal.backside, .3).wait_for_result()
        distance = getResult(getDistanceAction(goal.backside)).distance
        moveAction(distance, -.2).wait_for_result()
        if self._as.is_preempt_requested():
            rospy.loginfo('Preempted Buoy Task')
            self._as.set_preempted()
            return
        self.xPub.publish(20, LinearCommand.FORCE)
        rospy.sleep(4)
        self.xPub.publish(0, LinearCommand.FORCE)

        rospy.loginfo("Finished Buoy task")

        # Back up after finishing task
        moveAction(-distance, 0.2).wait_for_result()

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('buoy_task')
    server = BuoyTaskAction()
    rospy.spin()
