#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String, Header
from riptide_msgs.msg import Depth
from riptide_msgs.msg import PwmStamped
from riptide_msgs.msg import SwitchState

COM_PORT = '/dev/copro'
ser = serial.Serial(COM_PORT, baudrate=9600, timeout=None)

def pwm_callback(pwm_message):
    #The Start and End bytes for a PWM Message
    pwmStart = "####"
    pwmEnd = "@@@@"

    #Each thruster's pwm value is stored
    ssh = str(pwm_message.pwm.surge_stbd_hi)
    sph = str(pwm_message.pwm.surge_port_hi)
    spl = str(pwm_message.pwm.surge_port_lo)
    ssl = str(pwm_message.pwm.surge_stbd_lo)
    swf = str(pwm_message.pwm.sway_fwd)
    swa = str(pwm_message.pwm.sway_aft)
    hpa = str(pwm_message.pwm.heave_port_aft)
    hsa = str(pwm_message.pwm.heave_stbd_aft)
    hsf = str(pwm_message.pwm.heave_stbd_fwd)
    hpf = str(pwm_message.pwm.heave_port_fwd)

    #The pwm values and start and end bytes are added to a String and written
    final_pwm = pwmStart + spl + ssl + swf + swa + hpa + hsa + hsf + hpf + ssh + sph + pwmEnd
    final_pwm = bytes(final_pwm)
    ser.write(final_pwm)

def main():
    rospy.init_node('coprocessor_serial')
    dataRead = True

    # Add publishers
    depthPub = rospy.Publisher('/state/depth', Depth, queue_size=10)
    swPub = rospy.Publisher('/state/switches', SwitchState, queue_size=10)

    #Subscribe to Thruster PWMs
    rospy.Subscriber("/command/pwm", PwmStamped, pwm_callback)

    packet = ""
    depthRead = False
    swRead = False
    depth_msg = Depth()
    sw_msg = SwitchState()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if ser is not None:
            data = ser.read()
            if data is not None:
                #First two conditions check for start bytes
                if (data == "%" and not swRead):
                    depthRead = True
		    packet=""
		elif (data == "$" and not depthRead):
		    swRead = True
                    packet=""
                elif(data == '@'):
                    if(depthRead):
                       depthRead = False
		       #print packet
                       depthList = packet.split("!")
		       #print depthList
		       depth_msg.header.stamp = rospy.Time.now()
		       depth_msg.temp = float(depthList[0].replace("\x00", ""))
                       depth_msg.pressure = float(depthList[1].replace("\x00", ""))
                       depth_msg.depth = float(depthList[2].replace("\x00",""))
		       depth_msg.altitude = 0.0
                       depthPub.publish(depth_msg);
		    elif(swRead):
			#print packet
			swRead = False
			# Populate switch message. Start at 1 to ignore line break
			sw_msg.kill = True if packet[0] is '1' else False
			sw_msg.sw1 = True if packet[1] is '1' else False
			sw_msg.sw2 = True if packet[2] is '1' else False
			sw_msg.sw3 = True if packet[3] is '1' else False
			sw_msg.sw4 = True if packet[4] is '1' else False
			sw_msg.sw5 = True if packet[5] is '1' else False
			swPub.publish(sw_msg)
		    packet = ""
                else:
                    packet = packet + data

        rate.sleep()

if __name__ == "__main__": main()
