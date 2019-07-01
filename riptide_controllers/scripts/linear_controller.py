#! /usr/bin/env python

import rospy
from riptide_msgs.msg import LinearCommand
from nortek_dvl.msg import Dvl
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Float32, Header
from dynamic_reconfigure.server import Server
from riptide_controllers.cfg import LinearControllerConfig
import math

class LinearController():

    VELOCITY_P = 2.0
    DRAG_COEFF = 0

    velocityCmd = None
    force = 0

    def cmdCb(self, msg):
        if msg.mode == LinearCommand.VELOCITY:
            self.velocityCmd = msg.value
        elif msg.mode == LinearCommand.FORCE:
            self.force = msg.value
            self.velocityCmd = None

    def updateState(self, velocity):

        # If there is a desired velocity
        if self.velocityCmd != None:
            # Set force porportional to velocity error
            if not math.isnan(velocity):
                self.force = self.VELOCITY_P * (self.velocityCmd - velocity) + self.DRAG_COEFF * velocity * abs(velocity)

    def reconfigure(self, config, name):
        self.VELOCITY_P = config[name + "_velocity_p"]
        self.DRAG_COEFF = config[name + "_drag_coeff"]
        
        

xController = LinearController()
yController = LinearController()

XPub = rospy.Publisher("/command/force_x", Float64, queue_size=5)
YPub = rospy.Publisher("/command/force_y", Float64, queue_size=5)

def dvlCb(msg):
    xController.updateState(msg.velocity.x)
    yController.updateState(msg.velocity.y)

    # Publish new forces
    XPub.publish(xController.force)
    YPub.publish(yController.force)


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
