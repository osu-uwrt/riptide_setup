#!/usr/bin/env python

import rospy
import socket
import select
import traceback
from threading import Thread
from collections import deque
from std_msgs.msg import String, Header, Bool
from riptide_msgs.msg import Depth
from riptide_msgs.msg import PwmStamped
from riptide_msgs.msg import StatusLight
from riptide_msgs.msg import SwitchState

IP_ADDR = '192.168.1.42'
copro = None
connected = False
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
    global connected
    try:
        copro = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        copro.settimeout(timeout)
        copro.connect((IP_ADDR, 50000))
        connected = True
        return True
    except:
        connected = False
        copro = None
        return False

def enqueueCommand(command, args = []):
    global command_queue
    global response_queue
    
    data = [command] + args
    data = [len(data) + 1] + data
    command_queue.append(data)
    response_queue.append(command)

def toBytes(num):
    return [num // 256, num % 256]

def pwm_callback(pwm_message):
    global command_queue
    global response_queue

    args = []
    args += toBytes(pwm_message.pwm.heave_stbd_aft)
    args += toBytes(pwm_message.pwm.heave_stbd_fwd)
    args += toBytes(pwm_message.pwm.vector_stbd_fwd)
    args += toBytes(pwm_message.pwm.vector_stbd_aft)
    args += toBytes(pwm_message.pwm.heave_port_fwd)
    args += toBytes(pwm_message.pwm.heave_port_aft)
    args += toBytes(pwm_message.pwm.vector_port_fwd)
    args += toBytes(pwm_message.pwm.vector_port_aft)
    enqueueCommand(7, args)
    
def depth_callback(event):
    if depth_pub.get_num_connections() > 0:
        enqueueCommand(11)

def switch_callback(event):
    if switch_pub.get_num_connections() > 0:
        enqueueCommand(10)
    
def shutdown_copro():
    if connected:
        # disable thrusters
        copro.sendall(bytearray([3, 2, 0]))
        copro.sendall(bytearray([0]))
        copro.close()

def main():
    global copro
    global depth_pub
    global switch_pub
    global connection_pub
    global connected
    global buffer

    rospy.init_node('coprocessor_serial')

    # add publishers
    depth_pub = rospy.Publisher('/depth/raw', Depth, queue_size=1)
    switch_pub = rospy.Publisher('/state/switches', SwitchState, queue_size=1)
    connection_pub = rospy.Publisher('/state/copro', Bool, queue_size=1)

    rospy.Subscriber('/command/pwm', PwmStamped, pwm_callback, queue_size=1)
    # rospy.Subscriber('/status/light', StatusLight, light_callback, queue_size=1)
    
    # setup timer for periodic depth/switches update
    rospy.Timer(rospy.Duration(0.05), depth_callback)
    rospy.Timer(rospy.Duration(0.2), switch_callback)

    # 200 Hz rate for checking for messages to send
    rate = rospy.Rate(100)
    
    # set up clean shutdown
    rospy.on_shutdown(shutdown_copro)

    while not rospy.is_shutdown():
        if connected:
            try:
                readable, writable, exceptional = select.select([copro], [copro], [copro], 0)

                if len(writable) > 0 and len(command_queue) > 0:
                    # send pwm commands
                    command = []
                    while len(command_queue) != 0:
                        command += command_queue.popleft()
                    copro.sendall(bytearray(command))
                if len(readable) > 0:
                    # read the switch or depth data and publish
                    buffer += copro.recv(1024)
                    if not isinstance(buffer[0], int):
                        buffer = list(map(ord, buffer))
                    while len(buffer) > 0 and buffer[0] <= len(buffer):
                        response = buffer[1:buffer[0]]
                        command = response_queue.popleft()

                        if command == 11: # depth command
                            if len(response) != 3:
                                print("Improper depth response: " + str(response))
                            else:
                                depth = (response[0] << 16) + (response[1] << 8) + response[2]
                                if response[0] & 0x80 != 0:
                                    depth = -1 * ((1<<24) - depth)
                                depth = depth / 100000.0
                                depth_msg = Depth()
                                depth_msg.header.stamp = rospy.Time.now()
                                depth_msg.depth = depth
                                depth_pub.publish(depth_msg)

                        elif command == 10: # switches command
                            if len(response) != 1:
                                print("Improper switches response: " + str(response))
                            else:
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

            except Exception as e:
                traceback.print_exc()
                print(e)
                command_queue.clear()
                response_queue.clear()
                copro.close()
                copro = None
                connected = False
                
        else:
            print("Connecting to copro...")
            while not connect(1.0):
                connection_pub.publish(False)
                if rospy.is_shutdown():
                    return
            print("Connected to copro!")
            
            connection_pub.publish(True)
            response_queue.clear()
            command_queue.clear()

            # enable the thrusters
            enqueueCommand(2, [1])

        rate.sleep()


if __name__ == '__main__': main()
