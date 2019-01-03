#ifndef POSE_EDKF_INTERFACE
#define POSE_EDKF_INTERFACE

//#include "riptide_gnc/pose_ekf.h"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "math.h"
//#include <iostream>

using namespace Eigen;
using namespace std;

class PoseEDKFInterface
{
  private:
  ros::NodeHandle nh;

  public:
    PoseEDKFInterface();
};

#endif