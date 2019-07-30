#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import AttitudeCommand, Imu
import riptide_controllers.msg

def angleDiff(a1, a2):
    return (a1 - a2 + 180) % 360 - 180

class GoToRollAction(object):

    def __init__(self):
        self.rollPub = rospy.Publisher("/command/roll", AttitudeCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer("go_to_roll", riptide_controllers.msg.GoToRollAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        rospy.loginfo("Going to Roll " + str(goal.roll)+ " deg")
        self.rollPub.publish(goal.roll, AttitudeCommand.POSITION)

        while abs(angleDiff(rospy.wait_for_message("/state/imu", Imu).rpy_deg.x, goal.roll)) > 5:
            rospy.sleep(0.05)

            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Go To Roll')
                self._as.set_preempted()
                return

        rospy.loginfo("At Roll")
        self._as.set_succeeded()
        
        
if __name__ == '__main__':
    rospy.init_node('go_to_roll')
    server = GoToRollAction()
    rospy.spin()