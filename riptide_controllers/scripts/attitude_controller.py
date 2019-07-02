#!/usr/bin/env python

import rospy
from riptide_msgs.msg import Depth, AttitudeCommand, Imu
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Vector3Stamped, Vector3
from dynamic_reconfigure.server import Server
from riptide_controllers.cfg import AttitudeControllerConfig
import math

D2R = 3.14159265/180

class RotationController():

    MAX_VELOCITY = 1.0
    DECEL_RATE = 1.0
    VELOCITY_P = 1.0
    DRAG_COEFF = 0

    positionCmd = None
    velocityCmd = None
    moment = 0

    def cmdCb(self, msg):
        if msg.mode == AttitudeCommand.POSITION:
            self.positionCmd = msg.value
        elif msg.mode == AttitudeCommand.VELOCITY:
            self.velocityCmd = min(max(-self.MAX_VELOCITY, msg.value), self.MAX_VELOCITY)
            self.positionCmd = None
        elif msg.mode == AttitudeCommand.MOMENT:
            self.moment = msg.value
            self.velocityCmd = None
            self.positionCmd = None

    def updateState(self, position, velocity):
        # If there is desired position
        if self.positionCmd != None:
            # Set velocity porportional to position error
            error = ((self.positionCmd - position + 180 + 360) % 360) - 180
            self.velocityCmd = self.DECEL_RATE * error
            self.velocityCmd = min(max(-self.MAX_VELOCITY, self.velocityCmd), self.MAX_VELOCITY)

        # If there is a desired velocity
        if self.velocityCmd != None:
            # Set moment porportional to velocity error
            self.moment = self.VELOCITY_P * (self.velocityCmd - velocity) + self.DRAG_COEFF * velocity * abs(velocity)

    def reconfigure(self, config, name):
        self.MAX_VELOCITY = config[name + "_max_velocity"]
        self.DECEL_RATE = config[name + "_decel_rate"]
        self.VELOCITY_P = config[name + "_velocity_p"]
        self.DRAG_COEFF = config[name + "_drag_coeff"]
        

rollController = RotationController()
pitchController = RotationController()
yawController = RotationController()

momentPub = rospy.Publisher("/command/moment", Vector3Stamped, queue_size=5)

def imuCb(msg):
    r = msg.rpy_deg.x
    p = msg.rpy_deg.y
    y = msg.rpy_deg.z

    # Update state of each controller
    rollController.updateState(r, msg.ang_vel_deg.x)
    pitchController.updateState(p, msg.ang_vel_deg.y)
    yawController.updateState(y, msg.ang_vel_deg.z)

    # Publish new moments
    header = Header()
    header.stamp = rospy.Time.now()
    mx = rollController.moment
    my = pitchController.moment
    mz = yawController.moment
    x = mx - math.sin(p*D2R)*mz
    y = math.cos(r*D2R)*my + math.sin(r*D2R)*math.cos(p*D2R)*mz
    z = math.cos(r*D2R)*math.cos(p*D2R)*mz - math.sin(r*D2R)*my
    momentPub.publish(header, Vector3(x, y, z))

def dynamicReconfigureCb(config, level):
    # On dynamic reconfiguration
    rollController.reconfigure(config, "r")
    pitchController.reconfigure(config, "p")
    yawController.reconfigure(config, "y")
    return config


if __name__ == '__main__':

    rospy.init_node("attitude_controller")

    # Set subscribers
    rospy.Subscriber("/command/roll", AttitudeCommand, rollController.cmdCb)
    rospy.Subscriber("/command/pitch", AttitudeCommand, pitchController.cmdCb)
    rospy.Subscriber("/command/yaw", AttitudeCommand, yawController.cmdCb)
    rospy.Subscriber("/state/imu", Imu, imuCb)
    
    Server(AttitudeControllerConfig, dynamicReconfigureCb)

    rospy.spin()
