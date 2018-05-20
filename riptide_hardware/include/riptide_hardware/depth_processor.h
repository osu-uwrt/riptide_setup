#ifndef DEPTH_PROCESSOR_H
#define DEPTH_PROCESSOR_H
#define DEPTH_OFFSET 0.1
#define DEPTH_SLOPE 1


#include "ros/ros.h"
#include "riptide_msgs/Depth.h" 


class DepthProcessor
{

private:
  ros::NodeHandle nh;
  ros::Subscriber depth_sub;
  ros::Publisher state_depth_pub;

  int cycles;
  int c; //Center index for arrays
  int size; //Size of state array

  //datasmoothing array
  float ds[5];
  riptide_msgs::Depth state[5]; //five of these for the data smoothing
public:
  DepthProcessor();
  void DepthCB(const riptide_msgs::Depth::ConstPtr& msg);
  void smoothData();
  void loop();
};

#endif
