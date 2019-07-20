#!/usr/bin/env python

import rospy
from riptide_msgs.msg import AlignmentCommand, DepthCommand, LinearCommand, Depth
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Float64, Int32, Int8
from geometry_msgs.msg import Vector3
from dynamic_reconfigure.server import Server
from riptide_controllers.cfg import AlignmentControllerConfig

import math

class AlignmentController():
    MAX_FORCE = 40.0
    X_FORCE_P = 2.0
    Y_FORCE_P = 2.0
    DEPTH_FORCE_P = 10.0

    cam_width = 644
    cam_height = 482
    currentCam = 0

    x_error = 0
    y_error = 0
    z_error = 0

    width_ratio = 0
    watchdog_timer = None
    
    # Do the shutdown in action
    # time0 = time.time()

    def cmdCb(self, msg):
        self.object = msg.object
        self.width = msg.width_ratio
        if self.object == "":
            self.shutdown()

    def depthCb(self, msg):
        self.currentDepth = msg.depth

    def cameraSelectionCb(self, msg):
        if msg.data != self.currentCam:
            self.currentCam = msg.data
    
    def bboxCb(self, msg):
        for bbox in msg.bounding_boxes:
            if self.object == bbox.Class:
                if self.watchdog_timer is not None:
                    self.watchdog_timer.shutdown()
                self.watchdog_timer = rospy.Timer(rospy.Duration(0.5), self.shutdown())
                self.x_error = (bbox.xmin + bbox.xmax) / 2 - self.cam_width / 2
                self.y_error = (bbox.ymin + bbox.ymax) / 2 - self.cam_height / 2
                self.z_error = (bbox.xmax - bbox.xmin) - self.cam_width * self.width_ratio

                if self.currentCam == 0:
                    YPub.publish(self.x_error * self.Y_FORCE_P, LinearCommand.FORCE)
                    XPub.publish(-self.z_error * self.X_FORCE_P, LinearCommand.FORCE)
                    depthPub.publish(True, self.currentDepth + self.y_error * self.DEPTH_FORCE_P)
                else:
                    YPub.publish(self.x_error * self.Y_FORCE_P, LinearCommand.FORCE)
                    XPub.publish(-self.y_error * self.X_FORCE_P, LinearCommand.FORCE)
                    depthPub.publish(True, self.currentDepth - self.z_error * self.DEPTH_FORCE_P)

    def shutdown(self):
        self.x_error = 0
        self.y_error = 0
        self.z_error = 0

        YPub.publish(0, LinearCommand.FORCE)
        XPub.publish(0, LinearCommand.FORCE)
        depthPub.publish(True, self.currentDepth)

    def reconfigure(self, config, name):
        self.X_FORCE_P = config[name + "_x_force_p"]
        self.Y_FORCE_P = config[name + "_y_force_p"]
        self.DEPTH_FORCE_P = config[name + "_depth_force_p"]
        self.MAX_FORCE = config[name + "_max_force"]
        
alignmentController = AlignmentController()

XPub = rospy.Publisher("/command/x", Float64, queue_size=5)
YPub = rospy.Publisher("/command/y", Float64, queue_size=5)
depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=5)

   
def dynamicReconfigureCb(config, level):
    # On dynamic reconfiguration
    alignmentController.reconfigure(config, "alignment")
    return config

if __name__ == '__main__':

    rospy.init_node("alignment_controller")

    # Set subscribers
    rospy.Subscriber("/command/camera", Int8, cameraSelectionCb)
    rospy.Subscriber("/command/alignment", AlignmentCommand, alignmentController.cmdCb)
    rospy.Subscriber("/state/bboxes", BoundingBoxes, alignmentController.bboxCb)
    rospy.Subscriber("/state/depth", Depth, alignmentController.depthCb)
    
    Server(AlignmentControllerConfig, dynamicReconfigureCb)

    rospy.spin()
