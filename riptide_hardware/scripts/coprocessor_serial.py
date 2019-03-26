#!/usr/bin/env python

import rospy
import socket
import select
from threading import Thread
from collections import deque
from std_msgs.msg import String, Header, Bool
from riptide_msgs.msg import Depth
from riptide_msgs.msg import PwmStamped
from riptide_msgs.msg import StatusLight
from riptide_msgs.msg import SwitchState

IP_ADDR = '192.168.1.42'
copro = None
# only add byte arrays to this queue
command_queue = deque([], 50)
# after appending command to command_queue,
# append the command id to the response queue
response_queue = deque([], 50)
buffer = []

# globals definition
depth_pub = None
switch_pub = None
connection_pub = None

def connect(timeout):
    global copro
    try:
        copro = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        copro.settimeout(timeout)
        copro.connect((IP_ADDR, 50000))
        return True
    except:
        copro = None
        return False


def background_thread():
    global copro
    global buffer
    global depth_pub
    global switch_pub
    global connection_pub
    global command_queue
    global response_queue
    
    while True:
        if copro is not None:
            readable, writable, exceptional = select.select([copro], [copro], [copro], 0)

            if len(writable) > 0 and len(command_queue) > 0:
                # send pwm commands
                command = command_queue.popleft()
                copro.sendall(command)
            if len(readable) > 0:
                # read the switch or depth data and publish
                buffer += copro.recv(1024)
                buffer = list(map(ord, buffer))
                while len(buffer) > 0 and buffer[0] <= len(buffer):
                    response = buffer[1:buffer[0]]
                    command = response_queue.popleft()

                    if command == 11 and len(response) == 3: # depth command
                        depth = (response[0] << 16) + (response[1] << 8) + response[2]
                        depth = depth / 100000.0
                        depth_msg = Depth()
                        depth_msg.header.stamp = rospy.Time.now()
                        depth_msg.depth = depth
                        depth_pub.publish(depth_msg)

                    elif command == 10 and len(response) == 1: # switches command
                        switch_msg = SwitchState()
                        switch_msg.header.stamp = rospy.Time.now()
                        switch_msg.kill = True if response[0] & 32 else False
                        switch_msg.sw1 = True if response[0] & 16 else False
                        switch_msg.sw2 = True if response[0] & 8 else False
                        switch_msg.sw3 = True if response[0] & 4 else False
                        switch_msg.sw4 = True if response[0] & 2 else False
                        switch_msg.sw5 = True if response[0] & 1 else False
                        switch_pub.publish(switch_msg)

                    # can add the responses for other commands here in the future

                    buffer = buffer[buffer[0]:]

            if len(exceptional) > 0:
                command_queue.clear()
                response_queue.clear()
                copro.close()
                copro = None

        # add sleep if running too fast
        # sleep(0.001)

def pwm_callback(pwm_message):
    global command_queue
    global response_queue

    # change the order based on the receiving order
    data = []
    # command id = 7, length = 17 bytes
    data.append(bytearray([17, 7]))
    data.append(pwm_message.pwm.heave_stbd_aft.to_bytes(2, byteorder='big'))
    data.append(pwm_message.pwm.heave_stbd_fwd.to_bytes(2, byteorder='big'))
    data.append(pwm_message.pwm.vector_stbd_fwd.to_bytes(2, byteorder='big'))
    data.append(pwm_message.pwm.vector_stbd_aft.to_bytes(2, byteorder='big'))
    data.append(pwm_message.pwm.heave_port_fwd.to_bytes(2, byteorder='big'))
    data.append(pwm_message.pwm.heave_port_aft.to_bytes(2, byteorder='big'))
    data.append(pwm_message.pwm.vector_port_fwd.to_bytes(2, byteorder='big'))
    data.append(pwm_message.pwm.vector_port_aft.to_bytes(2, byteorder='big'))
    command_queue.append(bytearray(data))
    response_queue.append(7)

def main():
    global copro
    global depth_pub
    global switch_pub
    global connection_pub

    rospy.init_node('coprocessor_serial')

    print("Connecting to copro...")
    while not connect(1.0) and not rospy.is_shutdown():
        rospy.sleep(1)
    print("Connected!!!")

    # add publishers
    depth_pub = rospy.Publisher('/depth/raw', Depth, queue_size=1)
    switch_pub = rospy.Publisher('/state/switches', SwitchState, queue_size=1)
    connection_pub = rospy.Publisher('/state/copro', Bool, queue_size=1)

    rospy.Subscriber('/command/pwm', PwmStamped, pwm_callback, queue_size=1)
    # rospy.Subscriber('/status/light', StatusLight, light_callback, queue_size=1)

    # initalize and start background thread
    thread = Thread(name='connection daemon', target=background_thread)
    thread.daemon = True
    thread.start()

    # enable the thrusters
    command_queue.append(bytearray([3, 2, 1]))
    response_queue.append(2)

    # 20 Hz update rate for depth and switches (this is the max)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # request switch state and depth value
        command_queue.append(bytearray([3, 11, 0, 3, 10, 0]))
        response_queue.append(11)
        response_queue.append(10)
        rate.sleep()
    
    # disable the thrusters, if they are on
    command_queue.append(bytearray([3, 2, 0]))
    response_queue.append(2)


if __name__ == '__main__': main()
