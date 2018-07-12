#ifndef YOLO_PROCESSOR_H
#define YOLO_PROCESSOR_H

/* README
To install jsoncpp, do: sudo apt-get install libjsoncpp-dev
*/

#include "ros/ros.h"
#include "cmath"
#include "riptide_msgs/TaskInfo.h"
#include "riptide_msgs/Constants.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include <yaml-cpp/yaml.h>
#include "sensor_msgs/image_encodings.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
using namespace cv;
using namespace std;

class YoloProcessor
{
 private:
  ros::NodeHandle nh;
  ros::Subscriber darknet_bbox_sub, image_sub, task_info_sub;
  ros::Publisher task_bbox_pub, low_detections_pub, task_image_pub;

  darknet_ros_msgs::BoundingBoxes task_bboxes, low_detections;
  string camera_topics[2] = {"/forward/image_undistorted", "/downward/image_undistorted"};
  cv_bridge::CvImagePtr cv_ptr;
  Mat task_image;
  int top_margin, num_rows, offset, text_start[4];
  vector<Scalar> colors;
  Scalar margin_color;

  YAML::Node tasks;
  string task_file, task_name;
  int task_id, num_tasks, alignment_plane, last_alignment_plane, num_objects, num_thresholds;
  vector<string> object_names;
  vector<double> thresholds;

 public:
  YoloProcessor();
  void UpdateTaskInfo();
  template <typename T>
  void LoadParam(string param, T &var);
  void ImageCB(const sensor_msgs::ImageConstPtr& msg);
  void DarknetBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void TaskInfoCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg);
  void Loop();
};

#endif
