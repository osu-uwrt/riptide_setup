#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/ImuVerbose.h"
#include "std_msgs/Header.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "imu_3dm_gx4/MagFieldCF.h"
#include "math.h"

class IMUProcessor
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_filter_sub, imu_mag_sub;
  ros::Publisher imu_verbose_state_pub;
  ros::Publisher imu_state_pub;
  int cycles;
  int c; //Center index for arrays
  int size; //Size of state array
  float zero_ang_vel_thresh; //Threshold for zero angular veocity [deg/s]


  //Shorthand matrices for data smoothing
  //"av" = "Angular Velocity"
  //"la" = "Linear Acceleration"
  float av[3][7], la[3][7];
  int post_IIR_LPF_bandwidth, filter_rate;
  float dt, alpha, prev_ang_vel[3], prev_linear_accel[3];

  //0 = current state, 1 = one state ago, 2 = two states ago, etc.
  //Only velocities and accelerations will be smoothed
  riptide_msgs::ImuVerbose state[7]; //Used for calculations, debugging, etc.
  riptide_msgs::Imu imu_state; //Used for the controllers
  float magBX, magBY, magBZ, mBX, mBY, mBZ, mWX, mWY, heading;
  double latitude, longitude, altitude, declination;

  tf::Matrix3x3 R_b2w;
  tf::Vector3 tf;

public:
  IMUProcessor(char **argv);
  void LoadProperty(std::string, double &param);
  void LoadProperty(std::string, int &param);
  void MagCallback(const imu_3dm_gx4::MagFieldCF::ConstPtr& mag_msg);
  void Norm(float v1, float v2, float v3, float *x, float *y, float *z);
  void FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg);
  void CvtRad2Deg();
  void ProcessEulerAngles();
  void SmoothDataGauss();
  void SmoothDataIIR();
  void PopulateIMUState();
  void Loop();
};

#endif
