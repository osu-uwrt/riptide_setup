#!/usr/bin/env python
#******************************
#* ROS Action Server Template *
#******************************

#************
#* INCLUDES *
#************
# The following are required includes.
# Replace "TestAction" with your action name.
import roslib
import roslib.load_manifest('riptide_autonomy')
import rospy

from smach_ros import SimpleActionState

#Import Action files from the message
from riptide_msgs.msg import TestAction, TestGoal, TestFeedback, TestResult
import riptide_msgs.msg
import actionlib

#**************************
#* LIGHTS, CAMERA, ACTION *
#**************************


class TestAction:
    def __init__(self, name):

        rospy.loginfo('Starting TestAction')

        self.actionName = name

        # If reading data from another node, create a subscriber here
        #subscriber = rospy.Subscriber()

        # Declare messages used to publish action result and feedback
        self.feedbackMsg = TestFeedback()
        self.resultMsg = TestResult()

        # Check if task is completed from subscriber
        self.isTaskCompleted = False

        # Declare any other variables needed during action execution
        self.actionServer = actionlib.SimpleActionServer(
            self.actionName, riptide_msgs.msg.TestAction,
            execute_cb=self.executeCB, auto_start=False)
        self.actionServer.start()

    # Define callback functions (execute, analysis, goal, etc.)
    def executeCB(self, goal):

        start = rospy.get_rostime()
        current = rospy.get_rostime()

        while self.actionServer.is_active():
            # If the state machine times out an action, it will request
            # the server to be preempted. The server must then preempt itself.
            # First check this. Otherwise, you can run the action code.
            if self.actionServer.is_preempt_requested():
                # Add appropriate information to resultMsg
                # i.e. self.resultMsg.foundGoal = False

                # preempted == aborted (?)
                self.actionServer.set_aborted(self.resultMsg)

            elif current - start > rospy.Duration(5):
                # i.e. self.resultMsg.foundGoal = True
                self.actionServer.set_succeeded(self.resultMsg)

            else:
                # Python has different thread for checking subscriber callback.
        # Determine whether task completed based on callback.
                if self.isTaskCompleted:
                    # Populate resultMsg and set action server completed.
                    self.actionServer.set_succeeded()

                # Then run the action code! If the task runs into issues,
                # call actionServer.setAborted()

            current = rospy.get_rostime()

    def subscriberCB(data):
        self.isTaskCompleted = True

# Entry point for action server, simply creates and starts Action Server node.
if __name__ == '__main__':
    # Init sets the node name in rostopic. Filename sets node name.
    rospy.init_node('TestAction')

    # Class name must match above code.
    TestAction(rospy.get_name())
    rospy.spin()
