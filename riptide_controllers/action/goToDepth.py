#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import DepthCommand, Depth
import riptide_controllers.msg


class GoToDepthAction(object):

    def __init__(self):
        self.depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer("go_to_depth", riptide_controllers.msg.GoToDepthAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        rospy.loginfo("Going to depth " + str(goal.depth)+ "m")
        self.depthPub.publish(True, goal.depth)

        while abs(rospy.wait_for_message("/state/depth", Depth).depth - goal.depth) > 0.1:
            rospy.sleep(0.05)

            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Go To Depth')
                self._as.set_preempted()
                return

        rospy.loginfo("At Depth")
        self._as.set_succeeded()
        
        
if __name__ == '__main__':
    rospy.init_node('go_to_depth')
    server = GoToDepthAction()
    rospy.spin()