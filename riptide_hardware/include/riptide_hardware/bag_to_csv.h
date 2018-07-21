#ifndef ACOUSTICS_H
#define ACOUSTICS_H

#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include <vector>
#include <iostream>
#include <fstream>
#include "riptide_msgs/Acoustics.h"

using namespace std;

#define NUM_OF_COLLECTIONS 8192

class Acoustics
{
private:
  ros::NodeHandle nh;
  ros::Subscriber acoustics_sub;

  string username, file_name, ext;

public:
  Acoustics();

  template <typename T>
  void LoadParam(string param, T &var);
  void Callback(const riptide_msgs::Acoustics::ConstPtr& ac);
};

#endif
