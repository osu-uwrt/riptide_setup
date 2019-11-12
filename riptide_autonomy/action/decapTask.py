#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import LinearCommand, AlignmentCommand, Depth
from std_msgs.msg import Bool, Int8
from darknet_ros_msgs.msg import BoundingBoxes
import riptide_autonomy.msg

from actionTools import *

class DecapTaskAction(object):

    def __init__(self):
        self.armPub = rospy.Publisher("/command/arm", Bool, queue_size=1)
        self.firePub = rospy.Publisher("/command/fire", Int8, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "decap_task", riptide_autonomy.msg.DecapTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.timer = rospy.Timer(rospy.Duration(0.05), lambda _: checkPreempted(self._as))


    def execute_cb(self, goal):
        self.armPub.publish(True)
        alignAction("Decap", .25).wait_for_result()
        done = False
        while not done:
            boxes = rospy.wait_for_message("/state/bboxes", BoundingBoxes)
            for a in boxes.bounding_boxes:
                if a.Class == "Oval":
                    x = (a.xmin + a.xmax) / 2
                    done = True
            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Decap Task')
                self.armPub.publish(False)
                self._as.set_preempted()
                return
        alignAction("Oval", .3).wait_for_result()
        depth = rospy.wait_for_message("/state/depth", Depth).depth
        if x < 322:
            moveAction(0, 1.5).wait_for_result()
            performActions(
                depthAction(depth + .4),
                moveAction(.7, 0),
            )
            moveAction(0, -1.5).wait_for_result()
        else:
            moveAction(0, -1.5).wait_for_result()
            performActions(
                depthAction(depth + .4),
                moveAction(.7, 0),
            )
            moveAction(0, 1.5).wait_for_result()
        
        moveAction(-2, 0).wait_for_result()
        
        alignAction("Decap", .3).wait_for_result()
        alignAction("Heart", .3).wait_for_result()
        depth = rospy.wait_for_message("/state/depth", Depth).depth
        performActions(
            depthAction(depth - .1),
            moveAction(.8, .3)
        )
        if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Decap Task')
                self.armPub.publish(False)
                self._as.set_preempted()
                return
        self.firePub.publish(1)
        rospy.sleep(10)
        moveAction(-2, 0).wait_for_result()
        alignAction("Decap", .25, True).wait_for_result()
        alignAction("Oval", .3).wait_for_result()
        

        performActions(
            depthAction(depth - .1),
            moveAction(.8, .3)
        )
        if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Decap Task')
                self.armPub.publish(False)
                self._as.set_preempted()
                return
        self.firePub.publish(0)
        rospy.sleep(10)
        moveAction(-2, 0).wait_for_result()
        self.armPub.publish(False)

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('decap_task')
    server = DecapTaskAction()
    rospy.spin()
