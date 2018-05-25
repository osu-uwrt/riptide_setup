#!/usr/bin/env python

'''Array / control mapping:


0 - Left stick right -> -1, left right left -> 1
1 - Left stick up -> 1, left stick down -> -1
2 - Right stick right -> -1 right stick left -> 1
3 - Right stick up -> 1 right stick down -> -1

12 - Left trigger
13 - Right trigger 1 -> 0 -> -1

'''

import rospy
from sensor_msgs.msg import Joy
from riptide_msgs.msg import Pwm

pub = None

def controller_callback(joy_msg):

	FORWARD_THRUST_FACTOR = 150
	VERTICAL_THRUST_FACTOR = 150
	SWAY_FACTOR = 150

	verbose = True
	msg = Pwm()

	axes = joy_msg.axes

	left_stick_vertical = axes[1]

	right_stick_horizontal = axes[2]

	left_trigger = (-1 * axes[12] + 1) / 2
	right_trigger = (-1 * axes[13] + 1) / 2

	msg.surge_port_lo = 1500 + FORWARD_THRUST_FACTOR * left_stick_vertical
	msg.surge_stbd_lo = 1500 + FORWARD_THRUST_FACTOR * left_stick_vertical
	msg.sway_fwd = 1500 + SWAY_FACTOR * right_stick_horizontal
	msg.sway_aft = 1500 - SWAY_FACTOR * right_stick_horizontal
	msg.heave_port_fwd = 1500 + VERTICAL_THRUST_FACTOR * (right_trigger if right_trigger > left_trigger else -1 * left_trigger)
	msg.heave_stbd_fwd = 1500 + VERTICAL_THRUST_FACTOR * (right_trigger if right_trigger > left_trigger else -1 * left_trigger)
	msg.heave_port_aft = 1500 + VERTICAL_THRUST_FACTOR * (right_trigger if right_trigger > left_trigger else -1 * left_trigger)
	msg.heave_stbd_aft = 1500 + VERTICAL_THRUST_FACTOR * (right_trigger if right_trigger > left_trigger else -1 * left_trigger)

	if verbose:
		print "Left Stick vertical:", left_stick_vertical

		print "Right Stick horizontal: ", right_stick_horizontal

		print "Left trigger: ", left_trigger
		print "Right trigger: ", right_trigger

	pub.publish(msg)



if __name__ == '__main__':
	rospy.init_node('ps3_control')
	rospy.Subscriber('joy', Joy, controller_callback)
	pub = rospy.Publisher('command/pwm', Pwm, queue_size=1)
	rospy.spin()