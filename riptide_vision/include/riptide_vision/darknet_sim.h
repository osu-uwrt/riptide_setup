#ifndef DARKNET_SIM_H
#define DARKNET_SIM_H

#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
using namespace std;

class DarknetSim
{
 private:
  ros::NodeHandle nh;
  ros::Publisher darknet_sim_pub;

  darknet_ros_msgs::BoundingBoxes bboxes;

 public:
  DarknetSim();
  void DarknetPub();
  void Loop();
};

#endif
