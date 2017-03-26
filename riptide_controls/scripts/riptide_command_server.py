#!/usr/bin/env python


import rospy
from riptide_msgs.msg import ThrustStamped
from geometry_msgs.msg import Accel, Vector3
from std_msgs.msg import *

class RiptideConstants:
    ACCEL_COMMAND_TOPIC = "command/accel"

    # Accelerations for each axis in meters per second
    # TODO: This is an assumed value. Actual values need to be determined experimentally
    LINEAR_X_ACCEL = 1.0
    LINEAR_Y_ACCEL = 1.0
    LINEAR_Z_ACCEL = 1.0

    ANGULAR_X_ACCEL = 1.0
    ANGULAR_Y_ACCEL = 1.0
    ANGULAR_Z_ACCEL = 1.0

    COMMAND_TRANSLATE_X_POS = "tx+"
    COMMAND_TRANSLATE_Y_POS = "ty+"
    COMMAND_TRANSLATE_Z_POS = "tz+"
    COMMAND_ROTATE_X_CW = "rx+"
    COMMAND_ROTATE_Y_CW = "ry+"
    COMMAND_ROTATE_Z_CW = "rz+"

    COMMAND_TRANSLATE_X_NEG = "tx-"
    COMMAND_TRANSLATE_Y_NEG = "ty-"
    COMMAND_TRANSLATE_Z_NEG = "tz-"
    COMMAND_ROTATE_X_CCW  = "rx-"
    COMMAND_ROTATE_Y_CCW = "ry-"
    COMMAND_ROTATE_Z_CCW = "rz-"


accelUpdated = False
accel = Accel()
accel_pub = rospy.Publisher(RiptideConstants.ACCEL_COMMAND_TOPIC, Accel, queue_size=10)

def translateX(direction):

    accel.linear.x = RiptideConstants.LINEAR_X_ACCEL if direction > 0 else -1 * RiptideConstants.LINEAR_X_ACCEL
    accel.linear.y = 0
    accel.linear.z = 0

    accel.angular.x = 0
    accel.angular.y = 0
    accel.angular.z = 0

    accelUpdated = True

    commit()

    return -1

def translateY(direction):

    accel.linear.x = 0
    accel.linear.y = RiptideConstants.LINEAR_Y_ACCEL if direction > 0 else -1 * RiptideConstants.LINEAR_Y_ACCEL
    accel.linear.z = 0

    accel.angular.x = 0
    accel.angular.y = 0
    accel.angular.z = 0

    accelUpdated = True

    commit()

    return -1

def translateZ(direction):

    accel.linear.x = 0
    accel.linear.y = 0
    accel.linear.z = RiptideConstants.LINEAR_Z_ACCEL if direction > 0 else -1 * RiptideConstants.LINEAR_Z_ACCEL

    accel.angular.x = 0
    accel.angular.y = 0
    accel.angular.z = 0

    accelUpdated = True

    commit()

    return -1

def rotateX(direction):

    accel.linear.x = 0
    accel.linear.y = 0
    accel.linear.z = 0

    accel.angular.x = RiptideConstants.ANGULAR_X_ACCEL if direction > 0 else -1 * RiptideConstants.ANGULAR_X_ACCEL
    accel.angular.y = 0
    accel.angular.z = 0

    accelUpdated = True

    commit()

    return -1

def rotateY(direction):

    accel.linear.x = 0
    accel.linear.y = 0
    accel.linear.z = 0

    accel.angular.x = 0
    accel.angular.y = RiptideConstants.ANGULAR_Y_ACCEL if direction > 0 else -1 * RiptideConstants.ANGULAR_Y_ACCEL
    accel.angular.z = 0

    accelUpdated = True

    commit()

    return -1

def rotateZ(direction):

    accel.linear.x = 0
    accel.linear.y = 0
    accel.linear.z = 0

    accel.angular.x = 0
    accel.angular.y = 0
    accel.angular.z = RiptideConstants.ANGULAR_Z_ACCEL if direction > 0 else -1 * RiptideConstants.ANGULAR_Z_ACCEL

    accelUpdated = True

    commit()

    return -1

def stop():

    accel.linear.x = 0
    accel.linear.y = 0
    accel.linear.z = 0

    accel.angular.x = 0
    accel.angular.y = 0
    accel.angular.z = 0

    accelUpdated = True

    commit()

    return -1

def commit():
    if not rospy.is_shutdown():
        accel_pub.publish(accel)

def command_cb(command_str):
    cmd = command_str.data

    if cmd == RiptideConstants.COMMAND_TRANSLATE_X_POS:
        translateX(1)
    elif cmd == RiptideConstants.COMMAND_TRANSLATE_Y_POS:
        translateY(1)
    elif cmd == RiptideConstants.COMMAND_TRANSLATE_Z_POS:
        translateZ(1)
    elif cmd == RiptideConstants.COMMAND_ROTATE_X_CW:
        rotateX(1)
    elif cmd == RiptideConstants.COMMAND_ROTATE_Y_CW:
        rotateY(1)
    elif cmd == RiptideConstants.COMMAND_ROTATE_Z_CW:
        rotateZ(1)

    elif cmd == RiptideConstants.COMMAND_TRANSLATE_X_NEG:
        translateX(-1)
    elif cmd == RiptideConstants.COMMAND_TRANSLATE_Y_NEG:
        translateY(-1)
    elif cmd == RiptideConstants.COMMAND_TRANSLATE_Z_NEG:
        translateZ(-1)
    elif cmd == RiptideConstants.COMMAND_ROTATE_X_CCW:
        rotateX(-1)
    elif cmd == RiptideConstants.COMMAND_ROTATE_Y_CCW:
        rotateY(-1)
    elif cmd == RiptideConstants.COMMAND_ROTATE_Z_CCW:
        rotateZ(-1)

if __name__ == '__main__':
    rospy.init_node('riptide_command_server')
    rospy.Subscriber('command/command', String, command_cb)
    rospy.spin()
