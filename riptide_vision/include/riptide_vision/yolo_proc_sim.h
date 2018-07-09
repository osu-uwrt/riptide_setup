#ifndef YOLO_PROC_SIM_H
#define YOLO_PROC_SIM_H

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "riptide_msgs/TaskInfo.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
using namespace std;

class YoloProcSim
{
 private:
  ros::NodeHandle nh;
  ros::Publisher darknet_sim_pub, task_sim_pub;

  darknet_ros_msgs::BoundingBoxes bboxes;
  riptide_msgs::TaskInfo task_msg;
  string task_name;
  int alignment_plane, num_objects;
  //std_msgs::Int8 object_id[];
  double thresh;

 public:
  YoloProcSim();
  void InitMsgs();
  template <typename T>
  void LoadParam(string param, T &var);
  void DarknetPub();
  void TaskPub();
  void Loop();
};

#endif
