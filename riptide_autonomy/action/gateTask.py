import rospy
import actionlib

import riptide_autonomy.msg

from actionWrapper import *

class GateTaskAction(object):

    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            "gate_task", riptide_autonomy.msg.GateTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()


    def execute_cb(self, goal):
        alignAction("Gate").wait_for_result()
        if goal.isLeft:
            moveAction(0, -1).wait_for_result()
        else:
            moveAction(0, 1).wait_for_result()
        gateManeuverAction().wait_for_result()

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('gate_task')
    server = GateTaskAction()
    rospy.spin()
