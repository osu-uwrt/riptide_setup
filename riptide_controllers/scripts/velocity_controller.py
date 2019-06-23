#! /usr/bin/env python

import rospy
from riptide_msgs.msg import Dvl, LinearCommand
from geometry_msgs import Vector3
from std_msgs.msg import Float64, Float32
from dynamic_reconfigure.server import Server

class LinearController():

    VELOCITY_P = 1.0

    velocityCmd = None
    force = 0

    def cmdCb(self, msg):
        if msg.mode == LinearCommand.VELOCITY:
            self.velocityCmd = min(max(-self.MAX_VELOCITY, msg.value), self.MAX_VELOCITY)
        elif msg.mode == LinearCommand.FORCE:
            self.moment = msg.value
            self.velocityCmd = None

    def updateState(self, velocity):

        # If there is a desired velocity
        if self.velocityCmd != None:
            # Set force porportional to velocity error
            self.force = self.VELOCITY_P * (self.velocityCmd - velocity)

    def reconfigure(self, config, name):
        self.VELOCITY_P = config[name + "_velocity_p"]
        
        

xController = LinearController()
yController = LinearController()

XPub = rospy.Publisher("/command/force_x", Float64, queue_size=5)
YPub = rospy.Publisher("/command/force_y", Float64, queue_size=5)

def dvlCb(msg):
    xController.updateState(msg.velocity.x)
    yController.updateState(msg.velocity.y)

    # Publish new forces
    header = Header()
    header.stamp = rospy.Time.now()
    XPub.publish(header, Vector3(xController.force))
    YPub.publish(header, Vector3(yController.force))


def dynamicReconfigureCb(config, level):
    # On dynamic reconfiguration
    xController.reconfigure(config, "x")
    yController.reconfigure(config, "y")
    return config


if __name__ == '__main__':

    rospy.init_node("linear_controller")

    # Set subscribers
    rospy.Subscriber("/command/x", LinearCommand, xController.cmdCb)
    rospy.Subscriber("/command/y", LinearCommand, yController.cmdCb)
    rospy.Subscriber("/state/dvl", Dvl, dvlCb)
    
    Server(LinearControllerConfig, dynamicReconfigureCb)

    rospy.spin()
