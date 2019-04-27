#!/usr/bin/env python
PKG='riptide_controllers'

import unittest
import rospy
from time import sleep, time
from riptide_msgs.msg import DepthCommand
from geometry_msgs.msg import Vector3Stamped


class TestDepthController(unittest.TestCase):

    def test_disable_depth(self):
        rospy.init_node("test_depth_controller")
        self.force_sub = rospy.Subscriber("/command/force_depth", Vector3Stamped, self.forceCB)
        self.command_pub = rospy.Publisher("/command/depth", DepthCommand, queue_size=1)

        timeout = time() + 5 
        while self.command_pub.get_num_connections() == 0:
            sleep(.1)
            self.assertLess(time(), timeout, "Depth controller did not connect to command")

        self.vector = None
        self.command_pub.publish(False, 0)
        sleep(.1)
        self.assertIsNotNone(self.vector, "No force sent on disable")
        self.assertEquals(self.vector.x, 0, "X disable force not 0")
        self.assertEquals(self.vector.y, 0, "Y disable force not 0")
        self.assertEquals(self.vector.z, 0, "Z disable force not 0")

    def forceCB(self, data):
        self.vector = data.vector


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_depth_controller', TestDepthController)