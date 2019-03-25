#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "imu_3dm_gx4/FilterOutput.h"
using namespace std;

class IMUProcessor
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_filter_sub;
  ros::Publisher imu_state_pub;

  // IIR LPF Variables
  int filter_rate;
  double post_IIR_LPF_bandwidth, dt, alpha;
  geometry_msgs::Vector3 raw_ang_vel, raw_linear_accel, prev_ang_vel, prev_linear_accel;

  riptide_msgs::Imu state;

public:
  IMUProcessor();
  template <typename T>
  void LoadParam(string param, T &var);
  void FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg);
  void SmoothDataIIR();
};

#endif
