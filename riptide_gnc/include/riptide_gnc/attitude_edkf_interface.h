#ifndef ATTITUDE_EDKF_INTERFACE
#define ATTITUDE_EDKF_INTERFACE

#include "riptide_gnc/attitude_edkf.h"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;
using namespace std;

class AttitudeEDKFInterface
{
  private:
  ros::NodeHandle nh;

  public:
    AttitudeEDKFInterface();
};

#endif