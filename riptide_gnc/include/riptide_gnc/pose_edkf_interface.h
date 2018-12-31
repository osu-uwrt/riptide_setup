#ifndef POSE_EDKF_INTERFACE
#define POSE_EDKF_INTERFACE

#include "riptide_gnc/pose_edkf.h"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "math.h"

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