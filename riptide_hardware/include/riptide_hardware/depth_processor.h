#ifndef DEPTH_PROCESSOR_H
#define DEPTH_PROCESSOR_H
//#define DEPTH_OFFSET 0.1 // Save, these were used by arduino
//#define DEPTH_SLOPE 1

#include "ros/ros.h"
#include "riptide_msgs/Depth.h"
using namespace std;

class DepthProcessor
{

private:
  ros::NodeHandle nh;
  ros::Subscriber depth_sub;
  ros::Publisher depth_state_pub;

  // IIR LPF Variables
  double post_IIR_LPF_bandwidth, sensor_rate, dt, alpha, prev_depth;

  riptide_msgs::Depth depth_state;
public:
  DepthProcessor();
  template <typename T>
  void LoadParam(string param, T &var);
  void DepthCB(const riptide_msgs::Depth::ConstPtr& depth_msg);
  void SmoothDataIIR();
  void Loop();
};

#endif
