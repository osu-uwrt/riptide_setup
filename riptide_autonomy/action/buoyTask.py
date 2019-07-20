import rospy
import actionlib

from riptide_controllers.msg import LinearCommand
import riptide_autonomy.msg

from actionWrapper import *

class BuoyTaskAction(object):

    threeBuoySides = ["Groot", "Batman", "Fairy"]

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=1)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "buoy_task", riptide_autonomy.msg.BuoyTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()


    def execute_cb(self, goal):
        alignAction("Cutie", .4).wait_for_result()
        distance = getDistanceAction("Cutie").wait_for_result().distance
        moveAction(distance, 0).wait_for_result()
        self.xPub.publish(10, LinearCommand.FORCE)
        rospy.sleep(2)
        self.xPub.publish(0, LinearCommand.FORCE)

        moveAction(-4, 0).wait_for_result()
        self.yPub.publish(20, LinearCommand.FORCE)
        frontFace = next(x for x in self.threeBuoySides if not x == goal.backside)
        waitAction(frontFace, 5).wait_for_result()
        distance = getDistanceAction(frontFace).wait_for_result().distance
        arcAction(170, 10, distance).wait_for_result()

        alignAction(goal.backside, .4).wait_for_result()
        distance = getDistanceAction(goal.backside).wait_for_result().distance
        moveAction(distance, 0).wait_for_result()
        self.xPub.publish(10, LinearCommand.FORCE)
        rospy.sleep(2)
        self.xPub.publish(0, LinearCommand.FORCE)

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('buoy_task')
    server = BuoyTaskAction()
    rospy.spin()
