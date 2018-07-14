#ifndef TSLAM_H
#define TSLAM_H
//#define DEPTH_OFFSET 0.1 // Save, these were used by arduino
//#define DEPTH_SLOPE 1

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "riptide_msgs/TaskInfo.h"
#include <cmath>
using namespace std;

class Gate
{

private:
  ros::NodeHandle nh;
  ros::Subscriber go_sub, yolo_sub, abort_sub;


public:
  Gate();
  void Go(const riptide_msgs::TaskInfo::ConstPtr& task);
  void YoloCB(darknet_ros_msgs::BoundingBoxes bboxes);
  void Abort(const std_msgs::Empty::ConstPtr& data);
};

#endif