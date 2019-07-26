#!/usr/bin/env python

import rospy
import socket
import select
import traceback
from threading import Thread
from collections import deque
from std_msgs.msg import String, Header, Bool, Float32MultiArray, Int8
from riptide_msgs.msg import Depth, PwmStamped, StatusLight, SwitchState
from riptide_hardware.cfg import CoprocessorDriverConfig
from dynamic_reconfigure.server import Server

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
thruster_current_pub = None

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

def thruster_current_callback(event):
    if thruster_current_pub.get_num_connections() > 0:
        enqueueCommand(12)
    
def shutdown_copro():
    if connected:
        # Stop thrusters
        rospy.loginfo("Stopping thrusters")
        copro.sendall(bytearray([18, 7, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220]))
        copro.sendall(bytearray([0]))
        rospy.sleep(.1)
        copro.close()

def reconfigure_callback(config, level):
    for i in range(5):
        start = config["coil_%d_start1" % (i+1)]
        end = config["coil_%d_end1" % (i+1)]
        enqueueCommand(16, [i, start // 256, start % 256, end // 256, end % 256])

        start = config["coil_%d_start2" % (i+1)]
        end = config["coil_%d_end2" % (i+1)]
        enqueueCommand(16, [i+16, start // 256, start % 256, end // 256, end % 256])

    enqueueCommand(16, [5, 0, 0, config["standby"] // 256, config["standby"] % 256])
    enqueueCommand(16, [21, 0, 0, config["standby"] // 256, config["standby"] % 256])

    return config

def arm_callback(msg):
    if msg.data:
        enqueueCommand(16, [144])
    else:
        enqueueCommand(16, [128])

def fire_callback(msg):
    if msg.data == 0:
        enqueueCommand(16, [160])
    else:
        enqueueCommand(16, [176])

def grab_callback(msg):
    if msg.data == 0:
        enqueueCommand(16, [224, 6, 64])
    elif msg.data > 0:
        enqueueCommand(16, [224, 4, 0])
    else:
        enqueueCommand(16, [224, 8, 0])

def drop_callback(msg):
    if msg.data == 0:
        enqueueCommand(16, [192])
    else:
        enqueueCommand(16, [208])


    

def main():
    global copro
    global depth_pub
    global switch_pub
    global connection_pub
    global thruster_current_pub
    global connected
    global buffer

    rospy.init_node('coprocessor_driver')

    # add publishers
    depth_pub = rospy.Publisher('/depth/raw', Depth, queue_size=1)
    switch_pub = rospy.Publisher('/state/switches', SwitchState, queue_size=1)
    connection_pub = rospy.Publisher('/state/copro', Bool, queue_size=1)
    thruster_current_pub = rospy.Publisher('/state/thruster_currents', Float32MultiArray, queue_size=1)

    rospy.Subscriber('/command/pwm', PwmStamped, pwm_callback)
    rospy.Subscriber('/command/drop', Int8, drop_callback)
    rospy.Subscriber('/command/arm', Bool, arm_callback)
    rospy.Subscriber('/command/fire', Int8, fire_callback)
    rospy.Subscriber('/command/grabber', Int8, grab_callback)

    Server(CoprocessorDriverConfig, reconfigure_callback)
    
    # setup timer for periodic depth/switches update
    rospy.Timer(rospy.Duration(0.05), depth_callback)
    rospy.Timer(rospy.Duration(0.2), switch_callback)
    rospy.Timer(rospy.Duration(0.2), thruster_current_callback)

    # 200 Hz rate for checking for messages to send
    rate = rospy.Rate(100)
    
    # set up clean shutdown
    rospy.on_shutdown(shutdown_copro)

    while not rospy.is_shutdown():
        if connected:
            try:
                readable, writable, exceptional = select.select([copro], [copro], [copro], 0)

                if len(writable) > 0 and len(command_queue) > 0:
                    command = []
                    while len(command_queue) != 0:
                        command += command_queue.popleft()
                    copro.sendall(bytearray(command))
                if len(readable) > 0:
                    buffer += copro.recv(50)
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

                        elif command == 12:
                            if len(response) != 8:
                                print("Improper thruster current response: " + str(response))
                            else:
                                current_msg = Float32MultiArray(data = [x/25.0 for x in response])
                                thruster_current_pub.publish(current_msg)

                        # can add the responses for other commands here in the future

                        buffer = buffer[buffer[0]:]

            except Exception as e:
                traceback.print_exc()
                print(e)
                command_queue.clear()
                response_queue.clear()
                buffer = []
                shutdown_copro()
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

        rate.sleep()


if __name__ == '__main__': main()