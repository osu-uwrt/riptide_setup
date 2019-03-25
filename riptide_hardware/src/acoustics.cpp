#include "riptide_hardware/acoustics.h"
#include <thread>

// /command/acoustics
// 		enabled: Start reporting acoustics data, -1 means leave in current state
//		pingFrequency: Frequency to listen to, -1 means leave in current state
//  	fileName: When published, next recording will be saved under the name specified. "" will do nothing

// /test/acoustics: Load data saved under this name and perform the math on it

// /state/acoustics: Output of position data


//////////////////////////////////////////////////////////////////////////////////////////////////
// To install dependencies:									//
//	1. Log in https://pins.opalkelly.com/downloads. blaine141@gmail.com pass is usual	//
//	2. Download FrontPanel-Ubuntu16.04LTS-x64-5.0.2.tgz 					//
//    	3. Unzip, run install.sh as su and sudo copy API/libokFrontPanel.so file to /usr/lib	//
//	4. Download fftw-3.3.8.tar.gz from http://www.fftw.org/download.html			//
//	5. Run as su: ./configure, make, make install						//
//////////////////////////////////////////////////////////////////////////////////////////////////

#define PING_DURATION 2048
#define MAX_TIME_OFFSET 300
#define MAX_OFFSET 300
#define SAMPLE_LENGTH 512
#define SAMPLE_LENGTH_FINE 256
#define SAMPLE_FREQ 512000

int main(int argc, char **argv)
{
	init(argc, argv, "acoustics");
	Acoustics ac;
	spin();
}

Acoustics::Acoustics()
{
	nh = NodeHandle("acoustics");
	acoustics_pub = nh.advertise<AcousticsStatus>("/state/acoustics", 1);
	attitude_pub = nh.advertise<AttitudeCommand>("/command/attitude", 1);
	reset_pub = nh.advertise<ResetControls>("/controls/reset", 1);
	static auto acoustics_sub = nh.subscribe<AcousticsCommand>("/command/acoustics", 1, &Acoustics::CommandCB, this);
	static auto acoustics_test_sub = nh.subscribe<std_msgs::String>("/test/acoustics", 1, &Acoustics::TestCB, this);
	static auto heading_sub = nh.subscribe<ControlStatusAngular>("/status/controls/angular", 1, (boost::function<void(const ControlStatusAngular::ConstPtr &)>)[this](const ControlStatusAngular::ConstPtr &msg) {
		curHeading = msg->yaw.current;
	});
	InitializeFPGA();

	// Run the fft once to let it initialize
	fft(new double[SAMPLE_LENGTH], 0);
	ROS_INFO("Acoustics initialized");
}

bool Acoustics::InitializeFPGA()
{
	ROS_INFO("Attempting to connect to housing.");
	fpga = new okCFrontPanel;
	if (okCFrontPanel::NoError != fpga->OpenBySerial())
	{
		ROS_ERROR("Could not connect to housing");
		return false;
	}

	fpga->LoadDefaultPLLConfiguration();

	fpga->SetTimeout(100);

	return ConfigureFPGA();
}

bool Acoustics::ConfigureFPGA()
{
	ROS_INFO("Configuring");
	int error = fpga->ConfigureFPGA("../osu-uwrt/riptide_software/src/riptide_hardware/resources/acoustics/output_file.rbf");
	if (error != okCFrontPanel::NoError)
	{
		ROS_ERROR("Failed to configure housing: %s", okCFrontPanel::GetErrorString(error).c_str());
		return false;
	}
	ROS_INFO("Configured successfully!");
	return true;
}

double plot(double* data, int length)
{
	ofstream plotFile;
	plotFile.open("../plotData.txt", ofstream::out | ofstream::trunc);
	for(int i = 0; i < length; i++)
		plotFile << data[i] << "\n";
	plotFile.close();
	system("gnuplot --persist -e \"plot '../plotData.txt' with lines\"");
}

double unsignedToSigned(long val)
{
	if ((val & (1 << 23)) != 0)
		val = val | ~((1 << 24) - 1);

	return val / 3355443.2;
}

double restrictAngle(double angle)
{
	while (angle > 180)
		angle -= 360;
	while (angle < -180)
		angle += 360;
	return angle;
}

double *getSample(double *list, int index, int length = SAMPLE_LENGTH)
{
	double *sample = new double[length];
	for (int i = 0; i < length; i++)
		sample[i] = list[index + i];
	return sample;
}

int maxIndex(double *list, int length)
{
	int index = 0;
	for (int i = 0; i < length; i++)
		if (list[i] > list[index])
			index = i;
	return index;
}

double *Acoustics::fft(double *data, int frequency)
{
	static fftw_complex *in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * SAMPLE_LENGTH);
	static fftw_complex *out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * SAMPLE_LENGTH);
	static fftw_plan p = fftw_plan_dft_1d(SAMPLE_LENGTH, in, out, FFTW_FORWARD, FFTW_MEASURE);

	for (int i = 0; i < SAMPLE_LENGTH; i++)
	{
		in[i][0] = data[i];
		in[i][1] = 0;
	}
	fftw_execute(p);
	double *output = new double[2];
	output[0] = out[frequency * SAMPLE_LENGTH / SAMPLE_FREQ][0];
	output[1] = out[frequency * SAMPLE_LENGTH / SAMPLE_FREQ][1];
	return output;
}

double magnitude(double *complex)
{
	return sqrt(complex[0] * complex[0] + complex[1] * complex[1]);
}

double phase(double *complex)
{
	return atan2(complex[1], complex[0]) * 180 / 3.14159265359;
}

void Acoustics::Calculate(double *PFdata, double *PAdata, double *SFdata, double *SAdata, int length)
{
	int period = SAMPLE_FREQ / pingFrequency;

	// Find highest amplitude of ping frequency to approximate ping time
	double *amps = new double[length / SAMPLE_LENGTH];

	for (int i = 0; i < length / SAMPLE_LENGTH - 1; i++)
		amps[i] = magnitude(fft(getSample(PFdata, SAMPLE_LENGTH * i), pingFrequency));

	int pingApprox = maxIndex(amps, length / SAMPLE_LENGTH) * SAMPLE_LENGTH;


	double max = 0;
	double PFTime = 0;
	double average = -1;
	double ratio = .99;

	for (int i = -PING_DURATION * 5 / 4; i < PING_DURATION * 1 / 4; i++)
	{
		double leading = magnitude(fft(getSample(PFdata, pingApprox + i), pingFrequency));
		double lagging = magnitude(fft(getSample(PFdata, pingApprox + i - SAMPLE_LENGTH), pingFrequency));

		double val = leading - lagging;

		if (average == -1)
			average = val;
		average = average * ratio + val * (1 - ratio);
		if (average > max)
		{
			max = average;
			PFTime = pingApprox + i;
		}
	}

	

	max = 0;
	double PATime = 0;
	average = -1;

	for (int i = -MAX_OFFSET; i < MAX_OFFSET; i++)
	{
		double leading = magnitude(fft(getSample(PAdata, (int)PFTime + i), pingFrequency));
		double lagging = magnitude(fft(getSample(PAdata, (int)PFTime + i - SAMPLE_LENGTH), pingFrequency));

		double val = leading - lagging;
		if (average == -1)
			average = val;
		average = average * ratio + val * (1 - ratio);
		if (average > max)
		{
			max = average;
			PATime = PFTime + i;
		}
	}


	max = 0;
	double SFTime = 0;
	average = -1;
	for (int i = -MAX_OFFSET; i < MAX_OFFSET; i++)
	{
		double leading = magnitude(fft(getSample(SFdata, (int)PFTime + i), pingFrequency));
		double lagging = magnitude(fft(getSample(SFdata, (int)PFTime + i - SAMPLE_LENGTH), pingFrequency));

		double val = leading - lagging;
		if (average == -1)
			average = val;
		average = average * ratio + val * (1 - ratio);
		if (average > max)
		{
			max = average;
			SFTime = PFTime + i;
		}
	}


	// Line up according to phase
	double PFphase = phase(fft(getSample(PFdata, (int)PFTime), pingFrequency));
	double PAphase = phase(fft(getSample(PAdata, (int)PATime), pingFrequency));
	double SFphase = phase(fft(getSample(SFdata, (int)SFTime), pingFrequency));

	double PFPAphaseDiff = restrictAngle(PFphase - PAphase) / 360 * period;
	double PFSFphaseDiff = restrictAngle(PFphase - SFphase) / 360 * period;
	PATime += PFPAphaseDiff;
	SFTime += PFSFphaseDiff;


	// Report result
	AcousticsStatus msg;
	msg.PFPADiff = PFTime - PATime;
	msg.PFSFDiff = PFTime - SFTime;

	double angle = atan2(SFTime - PFTime, PATime - PFTime) * 180 / 3.14159265359;
	msg.Angle = angle;
	ROS_INFO("Angle: %f", angle);

	acoustics_pub.publish(msg);

	AttitudeCommand cmd;
	cmd.roll_active = true;
	cmd.pitch_active = true;
	cmd.yaw_active = true;
	cmd.euler_rpy.x = 0;
	cmd.euler_rpy.y = 0;
	cmd.euler_rpy.z = restrictAngle(curHeading + angle);

	attitude_pub.publish(cmd);

	// Schedule next recording
	Time pingTime = collectionTime + Duration(PFTime / 512000.0) - Duration(.7); // Say the ping happened a half second earlier to center the ping in new colleciton window
	double timeSincePing = (Time::now() - pingTime).toSec();
	// Make sure timer duration will not go negative
	while (timeSincePing > 2)
		timeSincePing -= 2;

	// Collect at time of next ping
	static Timer t;
	t = nh.createTimer(
		Duration(2 - timeSincePing),
		(boost::function<void(const TimerEvent &)>)[this](const TimerEvent &event) {
			Collect(1000);
		},
		true);

	delete[] amps;
}

void Acoustics::Collect(int length)
{
	if (!enabled)
		return;

	if (!fpga->IsOpen())
	{
		ROS_ERROR("Housing not connected");
		return;
	}

	// Reset FIFOs
	fpga->SetWireInValue(0x00, 0x0004);
	fpga->UpdateWireIns();
	fpga->SetWireInValue(0x00, 0x0000);
	fpga->UpdateWireIns();

	ROS_INFO("Collecting");
	// Collect Data
	fpga->SetWireInValue(0x00, 0x0002);
	fpga->UpdateWireIns();

	collectionTime = Time::now();

	// Shedule the following code to run after the data has been recorded
	static Timer t;
	t = nh.createTimer(
		Duration(length / 1000.0),
		(boost::function<void(const TimerEvent &)>)[ this, length ](const TimerEvent &event) {
			ROS_INFO("Done");
			long err;
			int NumOfCollections = length * 512;
			unsigned char *data = new unsigned char[NumOfCollections * 16];

			// Stop recording and begin reading
			fpga->SetWireInValue(0x00, 0x0001);
			fpga->UpdateWireIns();
			err = fpga->ReadFromBlockPipeOut(0xa0, 512, NumOfCollections * 16, data);
			fpga->SetWireInValue(0x00, 0x0000);
			fpga->UpdateWireIns();

			if (err < 0)
			{
				ROS_ERROR("Reading Failed");
				return;
			}

			double *PFdata = new double[NumOfCollections],
				   *PAdata = new double[NumOfCollections],
				   *SFdata = new double[NumOfCollections],
				   *SAdata = new double[NumOfCollections];

			// Process data and save in arrays
			for (int i = 0; i < NumOfCollections; i++)
			{
				long SAVal = ((long)data[2 + 16 * i] << 16) + ((long)data[1 + 16 * i] << 8) + data[16 * i];
				double SAVoltage = unsignedToSigned(SAVal);
				long SFVal = ((long)data[5 + 16 * i] << 16) + ((long)data[4 + 16 * i] << 8) + data[3 + 16 * i];
				double SFVoltage = unsignedToSigned(SFVal);
				long PAVal = ((long)data[8 + 16 * i] << 16) + ((long)data[7 + 16 * i] << 8) + data[6 + 16 * i];
				double PAVoltage = unsignedToSigned(PAVal);
				long PFVal = ((long)data[11 + 16 * i] << 16) + ((long)data[10 + 16 * i] << 8) + data[9 + 16 * i];
				double PFVoltage = unsignedToSigned(PFVal);

				PFdata[i] = PFVoltage;
				PAdata[i] = PAVoltage;
				SFdata[i] = SFVoltage;
				SAdata[i] = SAVoltage;
			}

			// Save to file if requested
			if (fileName.length() != 0)
			{
				ROS_INFO("%s", fileName.c_str());
				char *PFqueue = new char[3 * NumOfCollections],
					 *PAqueue = new char[3 * NumOfCollections],
					 *SFqueue = new char[3 * NumOfCollections],
					 *SAqueue = new char[3 * NumOfCollections];

				for (int i = 0; i < NumOfCollections; i++)
				{
					PFqueue[3 * i] = data[16 * i];
					PFqueue[3 * i + 1] = data[16 * i + 1];
					PFqueue[3 * i + 2] = data[16 * i + 2];
					PAqueue[3 * i] = data[16 * i + 3];
					PAqueue[3 * i + 1] = data[16 * i + 4];
					PAqueue[3 * i + 2] = data[16 * i + 5];
					SFqueue[3 * i] = data[16 * i + 6];
					SFqueue[3 * i + 1] = data[16 * i + 7];
					SFqueue[3 * i + 2] = data[16 * i + 8];
					SAqueue[3 * i] = data[16 * i + 9];
					SAqueue[3 * i + 1] = data[16 * i + 10];
					SAqueue[3 * i + 2] = data[16 * i + 11];
				}

				ROS_INFO("Making directory");
				const int dir_err = mkdir(("../Saved Data/" + fileName).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
				ROS_INFO("Directory made");
				ofstream fout;
				fout.open(("../Saved Data/" + fileName + "/PF.dat").c_str(), ios::binary | ios::out);
				fout.write(PFqueue, 3 * NumOfCollections);
				fout.close();
				ROS_INFO("PF made");
				fout.open(("../Saved Data/" + fileName + "/PA.dat").c_str(), ios::binary | ios::out);
				fout.write(PAqueue, 3 * NumOfCollections);
				fout.close();
				fout.open(("../Saved Data/" + fileName + "/SF.dat").c_str(), ios::binary | ios::out);
				fout.write(SFqueue, 3 * NumOfCollections);
				fout.close();
				fout.open(("../Saved Data/" + fileName + "/SA.dat").c_str(), ios::binary | ios::out);
				fout.write(SAqueue, 3 * NumOfCollections);
				fout.close();

				fileName = "";
			}

			delete[] data;

			Calculate(PFdata, PAdata, SFdata, SAdata, NumOfCollections);

			delete[] PFdata;
			delete[] PAdata;
			delete[] SFdata;
			delete[] SAdata;
		},
		true);
}

void Acoustics::CommandCB(const AcousticsCommand::ConstPtr &msg)
{
	if (msg->enabled >= 0)
	{
		if (msg->enabled && !enabled)
		{
			ResetControls reset_msg;
			reset_msg.reset_surge = true;
			reset_msg.reset_sway = true;
			reset_msg.reset_heave = true;
			reset_msg.reset_roll = false;
			reset_msg.reset_pitch = false;
			reset_msg.reset_yaw = false;
			reset_msg.reset_depth = false;
			reset_msg.reset_pwm = false;
			reset_pub.publish(reset_msg);
			enabled = true;
			Collect(2000);
		}
		if (!msg->enabled)
			enabled = false;
	}

	if (msg->pingFrequency >= 0)
		pingFrequency = msg->pingFrequency;

	if (msg->fileName.length() != 0)
		fileName = msg->fileName;
}

void Acoustics::TestCB(const std_msgs::String::ConstPtr &msg)
{
	ifstream PFfile("../Saved Data/" + msg->data + "/PF.dat");
	char *PFbuffer = new char[3072000];
	PFfile.read(PFbuffer, 3072000);
	PFfile.close();
	ifstream PAfile("../Saved Data/" + msg->data + "/PA.dat");
	char *PAbuffer = new char[3072000];
	PAfile.read(PAbuffer, 3072000);
	PAfile.close();
	ifstream SFfile("../Saved Data/" + msg->data + "/SF.dat");
	char *SFbuffer = new char[3072000];
	SFfile.read(SFbuffer, 3072000);
	SFfile.close();

	double *PFdata = new double[1024000], *PAdata = new double[1024000], *SFdata = new double[1024000], *SAdata = new double[1024000];
	for (int i = 0; i < 1024000; i++)
	{
		long PFVal = ((long)((unsigned char)PFbuffer[2 + 3 * i]) << 16) + ((long)((unsigned char)PFbuffer[1 + 3 * i]) << 8) + (unsigned char)PFbuffer[3 * i];
		PFdata[i] = unsignedToSigned(PFVal);
		long PAVal = ((long)((unsigned char)PAbuffer[2 + 3 * i]) << 16) + ((long)((unsigned char)PAbuffer[1 + 3 * i]) << 8) + (unsigned char)PAbuffer[3 * i];
		PAdata[i] = unsignedToSigned(PAVal);
		long SFVal = ((long)((unsigned char)SFbuffer[2 + 3 * i]) << 16) + ((long)((unsigned char)SFbuffer[1 + 3 * i]) << 8) + (unsigned char)SFbuffer[3 * i];
		SFdata[i] = unsignedToSigned(SFVal);
	}

	collectionTime = Time::now();
	Calculate(PFdata, PAdata, SFdata, SAdata, 1024000);

	
}
