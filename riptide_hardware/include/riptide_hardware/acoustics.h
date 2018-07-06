#ifndef ACOUSTICS_H
#define ACOUSTICS_H

#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include <vector>
#include "okFrontPanelDLL.h"
#include <iostream>
#include <fstream>
#include "detectPhase.h"
#include "riptide_msgs/Acoustics.h"

using namespace std;

#define NUM_OF_COLLECTIONS 8192

class Acoustics
{
private:
  ros::NodeHandle nh;
  ros::Publisher acoustics_pub;
  okCFrontPanel *device;
  riptide_msgs::Acoustics acoustics_msg;

public:
  Acoustics();
  double DoubleMod(double c, int divisor);
  bool ConfigureFPGA();
  bool InitializeFPGA();
  void Collect();
  void Loop();
  std::string GetCurrentWorkingDir( void );
};

#endif
