#include "riptide_hardware/acoustics.h"


//!!!!!!!!!!!!!!!!!!!!!!!!!IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!///
// Must extract /resources/FrontPanel. Run install.sh as sudo,//
// and sudo copy API/okFrontPanel.so file to /usr/lib		  //
////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "acoustics");
  Acoustics ac;
  ac.Loop();
}

Acoustics::Acoustics() : nh("acoustics") { // NOTE: there is no namespace declared in nh()
	acoustics_pub = nh.advertise<riptide_msgs::Acoustics>("/state/acoustics", 1);
	InitializeFPGA();
}

bool Acoustics::InitializeFPGA()
{
	// Open the first XEM - try all board types.
	ROS_INFO("Attempting to open okCFrontPanel.");
	device = new okCFrontPanel;
	ROS_INFO("Attempting to open FPGA.");
	if (okCFrontPanel::NoError != device->OpenBySerial()) {
		ROS_ERROR("Device could not be opened.  Is one connected");
		return false;
	}

	device->LoadDefaultPLLConfiguration();

	device->SetTimeout(100);

	return ConfigureFPGA();
}

bool Acoustics::ConfigureFPGA()
{
	ROS_INFO("Configuring");
	int error = device->ConfigureFPGA("../osu-uwrt/riptide_software/src/riptide_hardware/resources/acoustics/output_file.rbf");
	if (error != okCFrontPanel::NoError) {
		ROS_ERROR("FPGA configuration failed: %s", okCFrontPanel::GetErrorString(error).c_str());
		return false;
	}
	ROS_INFO("FPGA configured successfully!");
	return true;
}


double Acoustics::DoubleMod(double c, int divisor) {
	while (c < 0 || c >= divisor) {
		if (c < 0) {
			c += divisor;
		}
		else {
			c -= divisor;
		}
	}
	return c;
}

void Acoustics::Collect() {
	if (!device->IsOpen())
	{
		ROS_ERROR("Device Not Open");
		InitializeFPGA();
		return;
	}

	device->ActivateTriggerIn(0x40, 0);

	unsigned char data[NUM_OF_COLLECTIONS * 16];


	long err = device->ReadFromBlockPipeOut(0xA0, 1024, 16 * NUM_OF_COLLECTIONS, data);
	if (err < 0)
	{
		ROS_ERROR("Read Error: %s", okCFrontPanel::GetErrorString(err).c_str());
		return;
	}

	double min = 2.5;
	double max = -2.5;

	int PFdata[NUM_OF_COLLECTIONS], PAdata[NUM_OF_COLLECTIONS], SFdata[NUM_OF_COLLECTIONS], SAdata[NUM_OF_COLLECTIONS];

	for (int i = 0; i < 4 * NUM_OF_COLLECTIONS; i++)
	{
		long val = ((long)data[2 + 4 * i] << 16) + ((long)data[1 + 4 * i] << 8) + data[4 * i];

		long signedVal = val;

		if ((val & (1 << 23)) != 0)
			signedVal = val | ~((1 << 24) - 1);

		double voltage = signedVal / 3355443.2;

		if (i < NUM_OF_COLLECTIONS) {
			PFdata[i] = val;
		}
		else if (i < NUM_OF_COLLECTIONS * 2) {
			PAdata[i - NUM_OF_COLLECTIONS] = val;
		}
		else if (i < NUM_OF_COLLECTIONS * 3) {
			SFdata[i - NUM_OF_COLLECTIONS * 2] = val;
		}
		else {
			SAdata[i - NUM_OF_COLLECTIONS * 3] = val;
		}

		if (max < voltage)
			max = voltage;

		if (min > voltage)
			min = voltage;
	}

	double PFphase, PAphase, SFphase, SAphase;
	double PFfreq, PAfreq, SFfreq, SAfreq;
	double PFamp, PAamp, SFamp, SAamp;

	double frequency = (PFfreq + PAfreq + SFfreq + SAfreq) / 4;

	detectPhase(PFdata, 2000, 2000, &PFphase, &PFfreq, &PFamp);
	detectPhase(PAdata, 2000, 2000, &PAphase, &PAfreq, &PAamp);
	detectPhase(SFdata, 2000, 2000, &SFphase, &SFfreq, &SFamp);
	detectPhase(SAdata, 2000, 2000, &SAphase, &SAfreq, &SAamp);

	double maxPhase = std::max(PFphase, std::max(SAphase, SFphase));
	double minPhase = std::min(PFphase, std::min(SAphase, SFphase));

	double median = (PFphase /*+ PAphase*/ + SAphase + SFphase - maxPhase - minPhase);

	PFphase = DoubleMod(PFphase - median + 180, 360);
	PAphase = DoubleMod(PAphase - median + 180, 360);
	SFphase = DoubleMod(SFphase - median + 180, 360);
	SAphase = DoubleMod(SAphase - median + 180, 360);

	double angle2 = asin((PFphase-SFphase)/20)*180/3.1415926535897;
	double angle3 = atan((PFphase-SFphase)/(SFphase-SAphase))*180/3.1415926535897;

	if(SFphase>SAphase)
		angle3 += 180;

	angle3 = DoubleMod(angle3, 360);

	double distanceBetween = -0.1;
	double a = PFphase / 360 * 1500 / frequency;
	double b = PAphase / 360 * 1500 / frequency;
	double c = SFphase / 360 * 1500 / frequency;
	double d = SAphase / 360 * 1500 / frequency;

	double w = (d*d+a*a-b*b-c*c)/(-2*(d+a-b-c));
	double x = (a*(a+2*w)-b*(b+2*w)+distanceBetween*distanceBetween)/(2*distanceBetween);
	double y = (a*(a+2*w)-c*(c+2*w)+distanceBetween*distanceBetween)/(2*distanceBetween);
	double z = sqrt(abs((a+w)*(a+w)-x*x-y*y));

    acoustics_msg.PFfreq = PFfreq;
	acoustics_msg.PAfreq = PAfreq;
	acoustics_msg.SFfreq = SFfreq;
	acoustics_msg.SAfreq = SAfreq;
	acoustics_msg.PFphase = PFphase;
	acoustics_msg.PAphase = PAphase;
	acoustics_msg.SFphase = SFphase;
	acoustics_msg.SAphase = SAphase;
	acoustics_msg.PFamp = PFamp;
	acoustics_msg.PAamp = PAamp;
	acoustics_msg.SFamp = SFamp;
	acoustics_msg.SAamp = SAamp;
	acoustics_msg.angle2 = angle2;
	acoustics_msg.angle3 = angle3;
	acoustics_msg.XPos = x;
	acoustics_msg.YPos = y;
	acoustics_msg.ZPos = z;

	acoustics_pub.publish(acoustics_msg);


}

void Acoustics::Loop()
{
  ros::Rate rate(1);
  while(!ros::isShuttingDown()) {
	Collect();
    ros::spinOnce();
    rate.sleep();
  }
}
