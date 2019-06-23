#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import Dvl, PositionCommand
from geometry_msgs import Vector3
from riptide_controllers.msg

class GoToPosition(object):

    def __init__(self):
        self.positionPub = rospy.Publisher("/command/position", PositionCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer("go_to_position", riptide_controllers.msg.GoToPosition, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        rospy.loginfo("Going to position " + str(goal.position)+ " in the robot frame")
        self.positionPub.publish(True, goal.position)

        while ( abs(rospy.wait_for_message("/state/dvl2", Dvl).vehicle_pos.x - goal.position.x) > 0.1 
                and abs(rospy.wait_for_message("/state/dvl2", Dvl).vehicle_pos.y - goal.position.y) > 0,1
                and abs(rospy.wait_for_message("/state/dvl2", Dvl).vehicle_pos.z - goal.position.z) > 0.1):
            rospy.sleep(0.05)

        rospy.loginfo("At desired position")
        self._as.set_succeeded()
        
        
if __name__ == '__main__':
    rospy.init_node('go_to_position')
    server = GoToPositionAction()
    rospy.spin()