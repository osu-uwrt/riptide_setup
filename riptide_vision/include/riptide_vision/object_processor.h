#ifndef OBJECT_PROCESSOR_H
#define OBJECT_PROCESSOR_H

#include "ros/ros.h"
#include "cmath"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/TaskInfo.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/Object.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/Imu.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include <yaml-cpp/yaml.h>
#include "sensor_msgs/image_encodings.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using namespace std;

class ObjectProcessor
{
 private:
  ros::NodeHandle nh;
  ros::Subscriber task_bbox_sub, image_sub, task_info_sub, alignment_cmd_sub;
  ros::Publisher object_pub, object_image_pub;

  darknet_ros_msgs::BoundingBox object_bbox;
  riptide_msgs::Object object;
  string camera_topics[2] = {"/forward/image_undistorted", "/downward/image_undistorted"};
  cv_bridge::CvImagePtr cv_ptr;
  int width, height, cam_center_x, cam_center_y;
  vector<Scalar> colors;

  geometry_msgs::Vector3 current_attitude;
  vector<double> object_headings;

  YAML::Node tasks;
  string task_file, task_name, object_name;
  int task_id, num_tasks, alignment_plane, last_alignment_plane, num_objects, num_thresholds;
  vector<string> object_names;
  vector<double> thresholds;

 public:
  ObjectProcessor();
  void UpdateTaskInfo();
  template <typename T>
  void LoadParam(string param, T &var);
  void ImageCB(const sensor_msgs::ImageConstPtr& msg);
  void TaskBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void TaskInfoCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg);
  void AlignmentCmdCB(const riptide_msgs::AlignmentCommand::ConstPtr& cmd);
};

#endif
