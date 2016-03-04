#!/usr/bin/env python

PACKAGE = 'riptide_navigation'
import roslib;roslib.load_manifest(PACKAGE)
import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {double_param}, {double_param}, {double_param}, {double_param}, {double_param}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("riptide_navigation", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1)
    x = 0
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0
        client.update_configuration({"double_param":(1/(x+1)), "double_param":(1/(x+1)), "double_param":(1/(x+1)), "double_param":(1/(x+1)), "double_param":(1/(x+1))})
        r.sleep()
