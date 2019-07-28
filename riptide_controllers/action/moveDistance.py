#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import LinearCommand
from nortek_dvl.msg import Dvl
from geometry_msgs.msg import Vector3
import riptide_controllers.msg

import math

class MoveDistance(object):
    P = 1
    MAX_VELOCITY = 1

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer("move_distance", riptide_controllers.msg.MoveDistanceAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        rospy.loginfo("Moving robot %fm x and %fm y" % (goal.x, goal.y))
        self.distanceX = 0
        self.distanceY = 0
        self.lastXVelocity = 0
        self.lastYVelocity = 0
        self.goal = goal
        dvl_sub = rospy.Subscriber("/state/dvl", Dvl, self.dvlCb)

        while abs(self.distanceX - goal.x) > 0.1 or abs(self.distanceY - goal.y) > 0.1:
            rospy.sleep(0.05)

            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Move Action')
                self.cleanup()
                self._as.set_preempted()
                return

        rospy.loginfo("At desired position")
        dvl_sub.unregister()
        self.xPub.publish(0, LinearCommand.VELOCITY)
        self.yPub.publish(0, LinearCommand.VELOCITY)
        rospy.sleep(0.5)
        self.cleanup()
        self._as.set_succeeded()

    def cleanup(self):
        self.xPub.publish(0, LinearCommand.FORCE)
        self.yPub.publish(0, LinearCommand.FORCE)


    def dvlCb(self, msg):
        if not math.isnan(msg.velocity.x):
            curXVel = msg.velocity.x
            curYVel = msg.velocity.y
        else:
            curXVel = self.lastXVelocity
            curYVel = self.lastYVelocity
        self.distanceX += (self.lastXVelocity + curXVel) / 2.0 / 8
        self.distanceY += (self.lastYVelocity + curYVel) / 2.0 / 8
        self.lastXVelocity = curXVel
        self.lastYVelocity = curYVel

        rospy.loginfo("X:%f Y:%f"%(self.distanceX, self.distanceY))

        velocityX = self.P * (self.goal.x - self.distanceX)
        velocityY = self.P * (self.goal.y - self.distanceY)

        velocityX = max(min(velocityX, self.MAX_VELOCITY), -self.MAX_VELOCITY)
        velocityY = max(min(velocityY, self.MAX_VELOCITY), -self.MAX_VELOCITY)

        self.xPub.publish(velocityX, LinearCommand.VELOCITY)
        self.yPub.publish(velocityY, LinearCommand.VELOCITY)
        
        
if __name__ == '__main__':
    rospy.init_node('move_distance')
    server = MoveDistance()
    rospy.spin()