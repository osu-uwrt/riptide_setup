#ifndef YOLO_PROCESSOR_H
#define YOLO_PROCESSOR_H

#include "ros/ros.h"
#include "cmath"
#include "riptide_msgs/TaskInfo.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/Object.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/Imu.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "geometry_msgs/Vector3"


#include <yaml-cpp/yaml.h>
#include "sensor_msgs/image_encodings.h"
#include "opencv3/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using namespace std;
typedef riptide_msgs::Constants rc;

class YoloProcessorNew
{
 private:
  ros::NodeHandle nh;
  ros::Subscriber darknet_bbox_sub, image_sub, task_info_sub;
  ros::Publisher object_pub, low_detections_pub, task_image_pub;

  darknet_ros_msgs::BoundingBoxes task_bboxes, low_detections, object_bbox;
  string camera_topics[2] = {"/stereo/left/image_rect_color", "/downward/image_rect_color"};
  riptide_msgs::Object object;
  cv_bridge::CvImagePtr cv_ptr;
  Mat task_image;
  int top_margin, num_rows, offset, text_start[4];
  int width, height, cam_center_x, cam_center_y;
  int task_id, num_tasks, alignment_plane, last_alignment_plane, num_objects, num_thresholds;
  vector<Scalar> colors;
  Scalar margin_color;
  geometry_msgs::Vector3 current_attitude;
  string task_file, task_name, object_name;
  vector<double> object_headings;
  vector<string> object_names;
  vector<double> thresholds;

 public:
  YoloProcessorNew();
  template <typename T>
  void LoadParam(string param, T &var);
  void UpdateTaskInfo();
  void ImageCB(const sensor_msgs::ImageConstPtr& msg);
  void DarknetBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void TaskBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void TaskInfoCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg);
  void AlignmentCmdCB(const riptide_msgs::AlignmentCommand::ConstPtr& cmd);
};

#endif