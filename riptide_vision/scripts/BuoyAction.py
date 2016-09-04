#!/usr/bin/env python
#******************************
#* ROS Action Server Template *
#******************************

#************
#* INCLUDES *
#************
# The following are required includes.
# Replace "BuoyAction" with your action name.
import roslib
import roslib.load_manifest('riptide_autonomy')
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from smach_ros import SimpleActionState

#Import Action files from the message
from riptide_msgs.msg import BuoyAction, BuoyGoal, BuoyFeedback, BuoyResult
import riptide_msgs.msg
import actionlib

#**************************
#* LIGHTS, CAMERA, ACTION *
#**************************


class BuoyAction:
    
    red = [(-15, 152, 152), (25, 255, 255)]
    green = [45, 75, 75), (85, 230, 230)]
    bridge = CvBridge()
    pub = rospy.Publisher('stereo/left/image_buoy', Image, queue_size=1)
    
    def __init__(self, name):

        self.actionName = name

        # If reading data from another node, create a subscriber here
        rospy.Subscriber('stereo/left/image_color', Image, imageCallback)

        # Declare messages used to publish action result and feedback
        self.feedbackMsg = BuoyFeedback()
        self.resultMsg = BuoyResult()

        # Check if task is completed from subscriber
        self.isTaskCompleted = False

        # Declare any other variables needed during action execution
        self.actionServer = actionlib.SimpleActionServer(
            self.actionName, riptide_msgs.msg.BuoyAction,
            execute_cb=self.executeCB, auto_start=False)
        self.actionServer.start()

    # Define callback functions (execute, analysis, goal, etc.)
    def executeCB(self, goal):
        
        if (goal.color == "red"):
          self.upper = red[0]
          self.lower = red[1]
        elif (goal.color == "green"):
          self.upper = green[0]
          self.lower = green[1]        

        while self.actionServer.is_active():
        
            if self.actionServer.is_preempt_requested():
              self.resultMsg.rammed = false
              self.actionServer.set_preempted(self.resultMsg)

            else:
              # Do checks for task completion and action code here  
              if self.isTaskCompleted:
                self.actionServer.set_succeeded(self.resultMsg)

    def imageCallback(image_message_in):
        cv_image = bridge.imgmsg_to_cv2(image_message_in, desired_encoding="bgr8")
        bounds, target = find_buoy(cv_image, self.lower, self.upper)
        
        image_message_out = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        pub.publish(image_message_out)
        
    def find_buoy(image, lower, upper):
      x = -1
      y = -1
      radius = -1
      center = -1

      # Remove non-targeted colors:
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      threshold = cv2.inRange(hsv, lower, upper)

      # Remove noise:
      kernel = np.ones((5,5), np.uint8)
      opening = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)
      closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

      # Identify contours
      contours, hierarchy = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      if len(contours) > 0:
      
          # Determine largest contour
          largest_contour = max(contours, key=cv2.contourArea)
          center = None
          # Find its bounding cirlce
          (x, y), radius = cv2.minEnclosingCircle(largest_contour)
          if not radius < 10:
              # Draw its bounding circle
              cv2.circle(image, (int(x), int(y)), int(radius), (255, 255, 255), 2)

              # Calculate its moments
              M = cv2.moments(largest_contour)
              if M["m00"] > 0:

                  # Calculate its center of mass
                  center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                  # Draw its center of mass
                  cv2.circle(image, center, 5, (0, 0, 0), -1)

      return ((x, y), radius), center

# Entry point for action server, simply creates and starts Action Server node.
if __name__ == '__main__':
    # Init sets the node name in rostopic. Filename sets node name.
    rospy.init_node('BuoyAction')

    # Class name must match above code.
    BuoyAction(rospy.get_name())
    rospy.spin()
