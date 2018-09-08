#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String, Header
from riptide_msgs.msg import Depth
from riptide_msgs.msg import PwmStamped
from riptide_msgs.msg import StatusLight
from riptide_msgs.msg import SwitchState

COM_PORT = '/dev/copro'
ser = serial.Serial(COM_PORT, baudrate=9600, timeout=None)

def pwm_callback(pwm_message):
    #The Start and End bytes for a PWM Message
    pwmStart = "####"
    pwmEnd = "@@@@"

    #Each thruster's pwm value is stored
    spl = str(pwm_message.pwm.surge_port_lo -20)
    ssl = str(pwm_message.pwm.surge_stbd_lo -20)
    swf = str(pwm_message.pwm.sway_fwd -20)
    swa = str(pwm_message.pwm.sway_aft -20)
    hpa = str(pwm_message.pwm.heave_port_aft -20)
    hsa = str(pwm_message.pwm.heave_stbd_aft -20)
    hsf = str(pwm_message.pwm.heave_stbd_fwd -20)
    hpf = str(pwm_message.pwm.heave_port_fwd -20)

    #The pwm values and start and end bytes are added to a String and written
    final_pwm = pwmStart + spl + ssl + swf + swa + hpa + hsa + hsf + hpf + pwmEnd
    final_pwm = bytes(final_pwm)
    ser.write(final_pwm)

def light_callback(light_msg):
    '''#The Start and End bytes for a PWM Message
    lightStart = "LLLL"
    lightEnd = "@@@@"

    #Each thruster's pwm value is stored
    g1 = str(int(light_msg.green1)
    r1 = str(int(light_msg.red1)
    g2 = str(int(light_msg.green2)
    r2 = str(int(light_msg.red2)
    g3 = str(int(light_msg.green3)
    r3 = str(int(light_msg.red3)
    g4 = str(int(light_msg.green4)
    r4 = str(int(light_msg.red4)

    #The pwm values and start and end bytes are added to a String and written
    final_light = lightStart + g1 + r1 + g2 + r2 + g3 + r3 + g4 + r4 + lightEnd
    final_light = bytes(final_light)
    #ser.write(final_light)'''

def main():
    rospy.init_node('coprocessor_serial')
    dataRead = True

    # Add publishers
    depthPub = rospy.Publisher('/depth/raw', Depth, queue_size=1) #publish raw for the depth processor
    swPub = rospy.Publisher('/state/switches', SwitchState, queue_size=1)

    #Subscribe to Thruster PWMs
    rospy.Subscriber("/command/pwm", PwmStamped, pwm_callback, queue_size=1)
    rospy.Subscriber("/status/light", StatusLight, light_callback, queue_size=1)

    packet = ""
    depthRead = False
    swRead = False
    depth_msg = Depth()
    sw_msg = SwitchState()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if ser is not None:
            data = ser.readline()[:-2]
            if data is not None and data[-4:] == "@@@@":
                packet = data[5:-4]

                # Check if depth (%) or switch ($)
                if (len(packet) > 0 and data[1] == "%"):
                    try:
                        depthList = packet.split("!");
                        depth_msg.header.stamp = rospy.Time.now()
                        depth_msg.temp = float(depthList[0].replace("\x00", ""))
                        depth_msg.pressure = float(depthList[1].replace("\x00", ""))
                        depth_msg.depth = float(depthList[2].replace("\x00",""))
                        depth_msg.altitude = 0.0
                        depthPub.publish(depth_msg)
                        pass
                    except:
                        pass
                elif (len(packet) > 0 and data[1] == "$"):
                    # Populate switch message. Start at 1 to ignore line break
                    try:
                        sw_msg.header.stamp = rospy.Time.now()
                        sw_msg.kill = True if packet[0] is '1' else False
                        sw_msg.sw1 = True if packet[1] is '1' else False
                        sw_msg.sw2 = True if packet[2] is '1' else False
                        sw_msg.sw3 = True if packet[3] is '1' else False
                        sw_msg.sw4 = True if packet[4] is '1' else False
                        sw_msg.sw5 = True if packet[5] is '1' else False
                        swPub.publish(sw_msg)
                        pass
                    except:
                        pass

                        

        rate.sleep()
    rospy.loginfo("Stopping thrusters")
    #The pwm values and start and end bytes are added to a String and written
    final_pwm = "####14801480148014801480148014801480@@@@"
    final_pwm = bytes(final_pwm)
    ser.write(final_pwm)

if __name__ == "__main__": main()
