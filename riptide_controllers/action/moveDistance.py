#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import Dvl, LinearCommand
from geometry_msgs.msg import Vector3
import riptide_controllers.msg

import math

class MoveDistance(object):
    P = .5
    MAX_VELOCITY = 1

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer("move_distance", riptide_controllers.msg.MoveDistanceAction, execute_cb=self.execute_cb, auto_start=False)
        self.distanceX = 0
        self.distanceY = 0
        self.lastXVelocity = 0
        self.lastYVelocity = 0
        self._as.start()

      
    def execute_cb(self, goal):
        rospy.loginfo("Moving robot %dm x and %dm y" % (goal.x, goal.y))
        self.goal = goal
        dvl_sub = rospy.Subscriber("/state/Dvl", Dvl, self.dvlCb)

        while abs(self.distanceX - goal.position.x) > 0.1 or abs(self.distanceY - goal.position.y) > 0.1:
            rospy.sleep(0.05)

        rospy.loginfo("At desired position")
        dvl_sub.unregister()
        self.xPub.publish(0, LinearCommand.VELOCITY)
        self.yPub.publish(0, LinearCommand.VELOCITY)
        rospy.sleep(0.5)
        self.xPub.publish(0, LinearCommand.FORCE)
        self.yPub.publish(0, LinearCommand.FORCE)
        self._as.set_succeeded()

    def dvlCb(self, msg):
        if not math.isnan(msg.velocity.x):
            self.lastXVelocity = msg.velocity.x
            self.lastYVelocity = msg.velocity.y
        self.distanceX += self.lastXVelocity * (1/8)
        self.distanceY += self.lastYVelocity * (1/8)

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