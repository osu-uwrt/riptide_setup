#ifndef BBOX_PUB_H
#define BBOX_PUB_H

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include "riptide_msgs/Constants.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
using namespace std;
typedef riptide_msgs::Constants rc;

class BBoxPub
{
 private:
  ros::NodeHandle nh;
  ros::Publisher bbox_pub;
  ros::Subscriber bbox_sub;



 public:
  BBoxPub();
  void BBoxCB(const darknet_ros_msgs::BoundingBox::ConstPtr &msg);
};

#endif
