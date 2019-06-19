#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import AttitudeCommand, Imu
import riptide_controllers.msg

def angleDiff(a1, a2):
    return (a1 - a2 + 180) % 360 - 180

class GoToPitchAction(object):

    def __init__(self):
        self.pitchPub = rospy.Publisher("/command/pitch", AttitudeCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer("go_to_pitch", riptide_controllers.msg.GoToPitchAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        rospy.loginfo("Going to Pitch " + str(goal.pitch)+ " deg")
        self.pitchPub.publish(goal.pitch, AttitudeCommand.POSITION)

        while abs(angleDiff(rospy.wait_for_message("/state/imu", Imu).rpy_deg.y, goal.pitch)) > 5:
            rospy.sleep(0.05)

        rospy.loginfo("At Pitch")
        self._as.set_succeeded()
        
        
if __name__ == '__main__':
    rospy.init_node('go_to_pitch')
    server = GoToPitchAction()
    rospy.spin()