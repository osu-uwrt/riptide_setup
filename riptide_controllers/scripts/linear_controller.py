#!/usr/bin/env python

import rospy
from nortek_dvl.msg import Dvl
from riptide_msgs.msg import LinearCommand Imu
from std_msgs.msg import Float32, Float64, Header
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Vector3
from riptide_controllers.cfg import LinearControllerConfig

class LinearController():

    CRUISE_VELOCITY = 1.0
    DECEL_RATE = 1.0
    VELOCITY_P = 1.0

    positionCmd = None
    velocityCmd = None
    force = 0

    def cmdCb(self, msg):
        if msg.mode == LinearCommand.POSITION:
            self.positionCmd = msg.value
        elif msg.mode == LinearCommand.VELOCITY:
            self.velocityCmd = msg.value
            self.positionCmd = None
        elif msg.mode == LinearCommand.FORCE:
            self.force = msg.value
            self.velocityCmd = None
            self.positionCmd = None

    def updateState(self, position, velocity):
        # If there is desired position
        if self.positionCmd != None:
            # Set velocity proportional to position error
            error = self.positionCmd * - position
            self.velocityCmd = self.DECEL_RATE * error
            self.velocityCmd = min(max(-self.CRUISE_VELOCITY, self.velocityCmd), self.CRUISE_VELOCITY)

        # If there is a desired velocity
        if self.velocityCmd != None:
            # Set force proportional to velocity error
            self.force = self.VELOCITY_P * (self.velocityCmd - velocity)

    def reconfigure(self, config, name):
        self.CRUISE_VELOCITY = config[name + "_cruise_velocity"]
        self.DECEL_RATE = config[name + "_decel_rate"]
        self.VELOCITY_P = config[name + "_velocity_p"]
        
        

linearXController = LinearController()
linearYController = LinearController()

forceXPub = rospy.Publisher("/command/force_x", Float64, queue_size=5)
forceYPub = rospy.Publisher("/command/force_y", Float64, queue_size=5)

def imuCb(msg):
    angle_x = msg.rpy_rad.x
    angle_y = msg.rpy_rad.y

def dvlCb(msg):
    # Update state of each controller
    linearXController.updateState(msg.velocity.x, msg.position.x)
    linearYController.updateState(msg.velocity.y, msg.position.y)

    # Publish new forces
    header = Header()
    header.stamp = rospy.Time.now()
    forceXPub.publish(header, linearXController.force)
    forceYPub.publish(header, linearYController.force)


def dynamicReconfigureCb(config, level):
    # On dynamic reconfiguration
    linearXController.reconfigure(config, "x")
    linearXController.reconfigure(config, "y")   
    return config


if __name__ == '__main__':

    rospy.init_node("linear_controller")

    # Set subscribers
    rospy.Subscriber("/command/x", LinearCommand, linearXController.cmdCb)
    rospy.Subscriber("/command/y", LinearCommand, linearYController.cmdCb)
    rospy.Subscriber("/state/dvl", Dvl, dvlCb)
    
    Server(LinearControllerConfig, dynamicReconfigureCb)

    rospy.spin()