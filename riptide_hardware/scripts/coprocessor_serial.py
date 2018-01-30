import serial
import rospy
from std_msgs.msg import String
from riptide_msgs import Depth

def main():
   COM_PORT = '/dev/ttyACM0'
   ser = serial.Serial(COM_PORT, baudrate=9600, timeout=None)
   dataRead = True;

   #Thruster PWMs
   pwmData ="" #String?
   if ser is not None:
       while dataRead:
           data = ser.read();
           #If data is not empty
           if data is not "":
               if (data == "#"):
                   #Start byte recieved
                   dataRead = True
               elif(data == "@"):
                   #End byte recieved
                   dataRead = False
               else:
                   pwmData = pwmData + data

   #Assuming that depthData includes the data minus the start/end bytes
   pwmList = pwmData.split("")

   #--------------------------------------------------------------------------

   while (rospy.ROS_OK):
       rospy.spin()
       

   #DEPTH SENSOR
   depthData ="" #String?
   if ser is not None:
       while dataRead:
           data = ser.read();
           #If data is not empty
           if data is not "":
               if (data == "%"):
                   #Start byte recieved
                   dataRead = True
               elif(data == "@"):
                   #End byte recieved
                   dataRead = False
               else:
                   depthData = depthData + data

   #Assuming that depthData includes the data minus the start/end bytes
   depthList = depthData.split("!")

   Depth msg
   msg.depth =
   msg.pressure =
   msg.temp =
   msg.altitude =

   pub.publish(msg); # /state/depth
