#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import DepthCommand, AttitudeCommand, LinearCommand, Imu, Object, Depth
from std_msgs.msg import String, Int32, Float32, Float64
from darknet_ros_msgs.msg import BoundingBoxes
import riptide_controllers.msg

import time

class WaitAction(object):

    def __init__(self):

        self.depthPub = rospy.Publisher(
            "/command/depth", DepthCommand, queue_size=5)
        self.rollPub = rospy.Publisher(
            "/command/roll", AttitudeCommand, queue_size=5)
        self.pitchPub = rospy.Publisher(
            "/command/pitch", AttitudeCommand, queue_size=5)
        self.yawPub = rospy.Publisher(  
            "/command/yaw", AttitudeCommand, queue_size=5)

        self._as = actionlib.SimpleActionServer(
            "wait", riptide_controllers.msg.WaitAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def performActions(self, *actions):
        for a in actions:
            a.wait_for_result()

    def depthAction(self, depth):
        client = actionlib.SimpleActionClient(
            "go_to_depth", riptide_controllers.msg.GoToDepthAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToDepthGoal(depth))
        return client

    def rollAction(self, angle):
        client = actionlib.SimpleActionClient(
            "go_to_roll", riptide_controllers.msg.GoToRollAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToRollGoal(angle))
        return client

    def pitchAction(self, angle):
        client = actionlib.SimpleActionClient(
            "go_to_pitch", riptide_controllers.msg.GoToPitchAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToPitchGoal(angle))
        return client

    def yawAction(self, angle):
        client = actionlib.SimpleActionClient(
            "go_to_yaw", riptide_controllers.msg.GoToYawAction)
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(riptide_controllers.msg.GoToYawGoal(angle))
        return client
    
    def execute_cb(self, goal):
        rospy.loginfo("Waiting for object %s", goal.object)
        rospy.loginfo("Start spining")        
        self.performActions(
            self.depthAction(0.5),
            self.rollAction(0),
            self.pitchAction(0),
        )
        self.yawPub.publish(30, AttitudeCommand.VELOCITY)
        # Wait until you see the object a few times
        count = 0
        lastTime = time.time()
        while count < goal.times:
            boxes = rospy.wait_for_message("/state/bboxes", BoundingBoxes)
            for a in boxes.bounding_boxes:
                if a.Class == goal.object:
                    if (time.time() - lastTime) < 0.3:
                        count += 1
                    else:
                        count = 0
                lastTime = time.time()

        rospy.loginfo("Found object %s", goal.object)
        # Stops spinning, holds depth
        self.yawPub.publish(0, AttitudeCommand.VELOCITY)
        rospy.sleep(5.0)
        self.performActions(
            self.depthAction(0),
            self.rollAction(0),
            self.pitchAction(0),
        )   
        self.rollPub.publish(0, AttitudeCommand.MOMENT)
        self.pitchPub.publish(0, AttitudeCommand.MOMENT)
        self.yawPub.publish(0, AttitudeCommand.MOMENT)
        self.depthPub.publish(False, 0)
        rospy.loginfo("Stop spinning")        
        
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('wait')
    server = WaitAction()
    rospy.spin()
