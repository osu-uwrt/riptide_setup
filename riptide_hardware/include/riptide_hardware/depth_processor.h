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
  riptide_msgs::Depth lastDepth;
public:
  DepthProcessor();
  void DepthCB(const riptide_msgs::Depth::ConstPtr& msg);
};

#endif
