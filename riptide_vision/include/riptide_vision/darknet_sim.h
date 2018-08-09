#ifndef DARKNET_SIM_H
#define DARKNET_SIM_H

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include "riptide_msgs/Constants.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
using namespace std;
typedef riptide_msgs::Constants rc;

class DarknetSim
{
 private:
  ros::NodeHandle nh;
  ros::Publisher darknet_sim_pub;

  string tasks_sim_file;
  darknet_ros_msgs::BoundingBoxes bboxes;
  YAML::Node tasks;

 public:
  DarknetSim();
  void LoadSimData();
  void DarknetPub();
  void Loop();
};

#endif
