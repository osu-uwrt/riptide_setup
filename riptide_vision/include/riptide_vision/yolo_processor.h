#ifndef YOLO_PROCESSOR_H
#define YOLO_PROCESSOR_H

#include "ros/ros.h"
#include "cmath"
#include "std_msgs/Int8.h"
#include "riptide_msgs/TaskInfo.h"
#include "riptide_msgs/TaskBBoxes.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
using namespace std;

class YoloProcessor
{
 private:
  ros::NodeHandle nh;
  ros::Subscriber darknet_sub, task_sub;
  ros::Publisher task_bbox_pub;

  riptide_msgs::TaskBBoxes task_bboxes;
  string task_name;
  int alignment_plane, num_objects;
  vector<int> object_id;
  double thresh;

 public:
  YoloProcessor();
  void InitMsgs();
  void DarknetCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void TaskCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg);
  void Loop();
};

#endif
