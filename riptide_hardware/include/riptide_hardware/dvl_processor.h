#ifndef DVL_PROCESSOR_H
#define DVL_PROCESSOR_H

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "nortek_dvl/Dvl.h"
#include "riptide_msgs/Imu.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <yaml-cpp/yaml.h>

#define PI 3.141592653

using namespace Eigen;
using namespace std;

class DVLProcessor 
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_state_sub, dvl_data_sub;
  ros::Publisher dvl_state_pub, dvl_data_pub;

  YAML::Node properties;
  string properties_file;
  Vector3d dvl_position;
  double psi;
  Vector3d relative_vel;
  geometry_msgs::Vector3 dvl_vel;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DVLProcessor(); 
  template<typename T>
  void LoadParam(string param, T &var);
  void LoadDVLProperties();
  void ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg);
  void DvlCB(const nortek_dvl::Dvl::ConstPtr &dvl_msg);
};

#endif