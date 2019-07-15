import rospy
import actionlib

import riptide_autonomy.msg
from riptide_controllers.msg import LinearCommand

from actionWrapper import *

class GoToFinalsAction(object):

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "go_to_finals", riptide_autonomy.msg.GoToFinalsAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def goToTask(self, task):
        yawAction(12).wait_for_result()
        self.xPub.publish(20, LinearCommand.FORCE)


    def execute_cb(self, goal):
        if goal.quadrant == 0:
            performActions(
                depthAction(2),
                rollAction(0),
                pitchAction(0),
                yawAction(59)
            )
            self.goToTask("Gate")
            waitAction("Gate", 10).wait_for_result()
            self.xPub(0, LinearCommand.FORCE)
            gateTaskAction().wait_for_result()
        

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('gate_task')
    server = GoToFinalsAction()
    rospy.spin()
