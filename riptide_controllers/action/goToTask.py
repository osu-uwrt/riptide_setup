#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import Imu, Depth, Dvl
import riptide_controllers.msg
import yaml
with open('task_info.yaml', 'r') as f:
    doc = yaml.load(f)

class GoToTaskAction(object):
    numTasks = len(doc["tasks"])
    count = 0

    def __init__(self):
        self.taskPub = rospy.Publisher("/state/task", Dvl, queue_size=1)
        self._as = actionlib.SimpleActionServer("go_to_task", riptide_controllers.msg.GoToTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        if count < numTasks:
            rospy.loginfo("Going to task " + doc["tasks"][count]["name"])
            self.taskPub.publish(goal.position, Dvl.vehicle_pos)

            while ( abs(rospy.wait_for_message("/state/dvl2", Dvl).vehicle_pos.x - goal.position.x) > 0.1 
                  and abs(rospy.wait_for_message("/state/dvl2", Dvl).vehicle_pos.y - goal.position.y) > 0.1
                  and abs(rospy.wait_for_message("/state/dvl2", Dvl).vehicle_pos.z - goal.position.z) > 0.1):

                  rospy.sleep(0.05)

            rospy.loginfo("At task position")
            count = count + 1
            self._as.set_succeeded()
  
        
if __name__ == '__main__':
    rospy.init_node('go_to_task')
    server = GoToTaskAction()
    rospy.spin()