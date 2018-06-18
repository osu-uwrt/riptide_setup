#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/ImuVerbose.h"
#include "std_msgs/Header.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "imu_3dm_gx4/MagFieldCF.h"
#include "math.h"
using namespace std;

class IMUProcessor
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_filter_sub, imu_mag_sub;
  ros::Publisher imu_verbose_state_pub;
  ros::Publisher imu_state_pub;

  // IIR LPF Variables
  int filter_rate;
  double post_IIR_LPF_bandwidth, dt, alpha;
  geometry_msgs::Vector3 prev_ang_vel, prev_linear_accel;

  // Output Messages
  riptide_msgs::ImuVerbose verbose_state;
  riptide_msgs::Imu imu_state;

  // heading Calculation Variables
  float magBX, magBY, magBZ, mBX, mBY, mBZ, mWX, mWY, heading;
  double declination;

  tf::Matrix3x3 R_b2w;
  tf::Vector3 tf;

public:
  IMUProcessor();
  template <typename T>
  void LoadParam(string param, T &var);
  void MagCallback(const imu_3dm_gx4::MagFieldCF::ConstPtr& mag_msg);
  void Norm(float v1, float v2, float v3, float *x, float *y, float *z);
  void FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg);
  void CvtRad2Deg();
  void ProcessEulerAngles();
  void SmoothDataIIR();
  void PopulateIMUState();
  void Loop();
};

#endif
