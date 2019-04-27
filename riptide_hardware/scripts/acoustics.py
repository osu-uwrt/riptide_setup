#!/usr/bin/env python

from lib import ok
import rospy
from riptide_msgs.msg import AcousticsCommand

fpga = None

def commandCB(command):
	if fpga.IsOpen():
		rospy.logerr("Housing not connected")
		return

	# Reset FIFOs
	fpga.SetWireInValue(0x00, 0x0004)
	fpga.UpdateWireIns()
	fpga.SetWireInValue(0x00, 0x0000)
	fpga.UpdateWireIns()

	rospy.loginfo("Collecting")
	# Collect Data
	fpga.SetWireInValue(0x00, 0x0002)
	fpga.UpdateWireIns()

	length = 2000

	rospy.sleep(rospy.Duration(length / 1000.0))
	
	rospy.loginfo("Done")
	NumOfCollections = length * 512
	data = bytearray(NumOfCollections * 16)

	# Stop recording and begin reading
	fpga.SetWireInValue(0x00, 0x0001)
	fpga.UpdateWireIns()
	err = fpga.ReadFromBlockPipeOut(0xa0, 512, data)
	fpga.SetWireInValue(0x00, 0x0000)
	fpga.UpdateWireIns()

	if err < 0:
		rospy.logerr("Reading Failed")
		return

	file = open(command.fileName, "wb")
	file.write(data)
	

def initFPGA():
	global fpga

	rospy.loginfo("Attempting to connect to housing...")
	fpga = ok.okCFrontPanel()
	if ok.okCFrontPanel.NoError != fpga.OpenBySerial(""):
		rospy.logerr("Could not connect to housing")
		return False

	fpga.LoadDefaultPLLConfiguration()
	fpga.SetTimeout(100)

	rospy.loginfo("Configuring")
	error = fpga.ConfigureFPGA("../osu-uwrt/riptide_software/src/riptide_hardware/assets/acoustics/output_file.rbf")
	if error != ok.okCFrontPanel.NoError:
		rospy.logerr("Failed to configure housing: " + ok.okCFrontPanel.GetErrorString(error))
		return False
	rospy.loginfo("Configured successfully!")
	return True


if __name__ == '__main__':
    rospy.init_node("acoustics")
    if initFPGA():
		rospy.Subscriber("command/acoustics", AcousticsCommand, commandCB)
		rospy.spin()