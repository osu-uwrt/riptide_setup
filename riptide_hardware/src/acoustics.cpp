#include "riptide_hardware/acoustics.h"


//!!!!!!!!!!!!!!!!!!!!!!!!!IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!//
// Must extract /resources/FrontPanel. Run install.sh, and 	 //
// copy API/okFrontPanel.so file to /usr/lib				 //
///////////////////////////////////////////////////////////////

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
	device = new okCFrontPanel;

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
	int error = device->ConfigureFPGA("../osu-uwrt/riptide_software/src/riptide_hardware/resources/acoustics/output_file.rbf");
	if (error != okCFrontPanel::NoError) {
		ROS_ERROR("FPGA configuration failed: %s", okCFrontPanel::GetErrorString(error).c_str());
		return false;
	}
	return true;
}


double Acoustics::DoubleMod(double c, int divisor) {
	while (c < 0 || c >= divisor) {
		if (c < 0) {
			c += 360;
		}
		else {
			c -= 360;
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

	detectPhase(PFdata, &PFphase, &PFfreq);
	detectPhase(PAdata, &PAphase, &PAfreq);
	detectPhase(SFdata, &SFphase, &SFfreq);
	detectPhase(SAdata, &SAphase, &SAfreq);

	//double maxPhase = std::max(PFphase, std::max(PAphase, SFphase));
	//double minPhase = std::min(PFphase, std::min(PAphase, SFphase));

	double median = PFphase;// + PAphase + SFphase - maxPhase - minPhase;

	PFphase = DoubleMod(PFphase - median + 180, 360);
	//PAphase = doubleMod(PAphase - median + 180, 360);
	SFphase = DoubleMod(SFphase - median + 180, 360);

	double angle = asin((PFphase-SFphase)/20)*180/3.1415926535897;

    acoustics_msg.PFfreq = PFfreq;
	acoustics_msg.PAfreq = PAfreq;
	acoustics_msg.SFfreq = SFfreq;
	acoustics_msg.SAfreq = SAfreq;
	acoustics_msg.PFphase = PFphase;
	acoustics_msg.PAphase = PAphase;
	acoustics_msg.SFphase = SFphase;
	acoustics_msg.SAphase = SAphase;
	acoustics_msg.angle = angle;
	acoustics_pub.publish(acoustics_msg);


}

void Acoustics::Loop()
{
  ros::Rate rate(4);
  while(!ros::isShuttingDown()) {
	Collect();
    ros::spinOnce();
    rate.sleep();
  }
}



