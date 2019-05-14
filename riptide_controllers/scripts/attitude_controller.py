#!/usr/bin/env python

import rospy
from riptide_msgs.msg import Depth, AttitudeCommand, Imu
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Vector3Stamped, Vector3
from dynamic_reconfigure.server import Server
from riptide_controllers.cfg import AttitudeControllerConfig

class RotationController():

    MAX_VELOCITY = 1.0
    DECEL_RATE = 1.0
    VELOCITY_P = 1.0

    positionCmd = 0
    velocityCmd = 0
    torque = 0

    def positionCmdCb(self, msg):
        self.positionCmd = msg.data

    def velocityCmdCb(self, msg):
        self.velocityCmd = msg.data

    def updateState(self, position, velocity):
        error = ((self.positionCmd - position + 180 + 360) % 360) - 180
        self.velocityCmd = self.DECEL_RATE * error
        self.velocityCmd = min(max(-self.MAX_VELOCITY, self.velocityCmd), self.MAX_VELOCITY)
        self.torque = self.VELOCITY_P * (self.velocityCmd - velocity)

    def reconfigure(self, config, name):
        self.MAX_VELOCITY = config[name + "_max_velocity"]
        self.DECEL_RATE = config[name + "_decel_rate"]
        self.VELOCITY_P = config[name + "_velocity_p"]
        
        

rollController = RotationController()
pitchController = RotationController()
yawController = RotationController()

torquePub = rospy.Publisher("/command/moment", Vector3Stamped, queue_size=5)

def imuCb(msg):
    rollController.updateState(msg.rpy_deg.x, msg.ang_vel_deg.x)
    pitchController.updateState(msg.rpy_deg.y, msg.ang_vel_deg.y)
    yawController.updateState(msg.rpy_deg.z, msg.ang_vel_deg.z)

    torquePub.publish(Header(), Vector3(rollController.torque, pitchController.torque, yawController.torque))

def dynamicReconfigureCb(config, level):
    rollController.reconfigure(config, "r")
    pitchController.reconfigure(config, "p")
    yawController.reconfigure(config, "y")
    return config


if __name__ == '__main__':

    rospy.init_node("attitude_controller")

    rospy.Subscriber("/command/roll/position", Float32, rollController.positionCmdCb)
    rospy.Subscriber("/command/roll/velocity", Float32, rollController.velocityCmdCb)
    rospy.Subscriber("/command/pitch/position", Float32, pitchController.positionCmdCb)
    rospy.Subscriber("/command/pitch/velocity", Float32, pitchController.velocityCmdCb)
    rospy.Subscriber("/command/yaw/position", Float32, yawController.positionCmdCb)
    rospy.Subscriber("/command/yaw/velocity", Float32, yawController.velocityCmdCb)
    rospy.Subscriber("/state/imu", Imu, imuCb)
    
    Server(AttitudeControllerConfig, dynamicReconfigureCb)

    rospy.spin()
