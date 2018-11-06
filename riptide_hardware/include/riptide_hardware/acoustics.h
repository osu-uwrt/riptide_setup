#ifndef ACOUSTICS_H
#define ACOUSTICS_H

#include "ros/ros.h"
#include <sys/stat.h> 
#include <fstream>
#include "fftw3.h"
#include "okFrontPanelDLL.h"
#include "riptide_msgs/AcousticsStatus.h"
#include "riptide_msgs/AcousticsCommand.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "std_msgs/String.h"

using namespace std;
using namespace ros;
using namespace riptide_msgs;

class Acoustics
{
private:
  NodeHandle nh;
  Publisher acoustics_pub, attitude_pub;
  okCFrontPanel *fpga;
  double curHeading = 0;
  bool enabled = false;
  long pingFrequency;
  string fileName = "";
  Time collectionTime;

public:
  Acoustics();
  bool ConfigureFPGA();
  bool InitializeFPGA();
  double* fft(double*, int);
  void Calculate(double*, double*, double*, double*, int);
  void Collect(int);
  void CommandCB(const AcousticsCommand::ConstPtr&);
  void TestCB(const std_msgs::String::ConstPtr&);
};

#endif
