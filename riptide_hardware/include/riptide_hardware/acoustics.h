#ifndef ACOUSTICS_H
#define ACOUSTICS_H

#include "ros/ros.h"
#include <sys/stat.h> 
#include <fstream>
#include "fftw3.h"
#include "okFrontPanelDLL.h"
#include "riptide_msgs/Acoustics.h"
#include "riptide_msgs/AcousticsCommand.h"
#include "std_msgs/String.h"

using namespace std;
using namespace ros;

class Acoustics
{
private:
  NodeHandle nh;
  Publisher acoustics_pub;
  okCFrontPanel *fpga;
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
  void CommandCB(const riptide_msgs::AcousticsCommand::ConstPtr&);
  void TestCB(const std_msgs::String::ConstPtr&);
};

#endif
