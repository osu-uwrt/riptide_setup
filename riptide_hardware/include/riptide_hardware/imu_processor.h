#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

#include "ros/ros.h"
#include "eigen_conversions/eigen_msg.h"
#include <tf/tf.h>
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include <yaml-cpp/yaml.h>
#include "math.h"

class IMUProcessor
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_filter_sub;
  ros::Publisher imu_state_pub;

  YAML::Node properties;
  std::string properties_file;

  // Linear Accel Variables
  Eigen::Vector3d imu_position, pqr, pqr_dot, prev_pqr1, prev_pqr2, linear_accel, raw_accel;
  geometry_msgs::Vector3 ang_accel;
  ros::Time prev_time;
  double dt;
  int init_count;

  riptide_msgs::Imu state;

  // IIR LPF Variables
  int filter_rate;
  double post_IIR_LPF_bandwidth, alpha;
  geometry_msgs::Vector3 raw_ang_vel, raw_linear_accel;
  //geometry_msgs::Vector3 prev_ang_vel1, prev_ang_vel2, ang_accel, prev_linear_accel;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUProcessor();
  template <typename T>
  void LoadParam(std::string param, T &var);
  void LoadIMUProperties();
  void FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg);
  //void SmoothDataIIR();
};

#endif