#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String
from riptide_msgs.msg import Depth
from riptide_msgs.msg import PwmStamped

COM_PORT = '/dev/ttyACM0'
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
    final_pwm = pwmStart + ssh + sph + spl + ssl + swf + swa + hpa + hsa + hsf + hpf + pwmEnd
    print(final_pwm)
    final_pwm = bytes(final_pwm)
    ser.write(final_pwm)

def main():
    rospy.init_node('coprocessor_serial')
    dataRead = True;

    #Subscribe to Thruster PWMs
    rospy.Subscriber("/command/pwm", PwmStamped, pwm_callback)
    rospy.spin()

    while not rospy.is_shutdown():

        #Publishing depth sensor data
        depthData =""
        if ser is not None:
            while dataRead:
                data = ser.read();
                if data is not "":
                    if (data == "%"):
                        #Start byte recieved
                        dataRead = True
                    elif(data == "@"):
                        #End byte recieved
                        dataRead = False
                    else:
                        depthData = depthData + data

        depthList = depthData.split("!")
        Depth msg
        msg.pressure = depthList[0]
        msg.temp = depthList[1]
        msg.depth = depthList[2]
        msg.altitude = depthList[3]
        pub = rospy.Publisher('/state/depth', std_msgs.msg.String, queue_size=10)
        pub.publish(msg);


if __name__ == "__main__": main()
