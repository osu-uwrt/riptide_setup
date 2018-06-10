#ifndef UNDISTORT_CAMERA_H
#define UNDISTORT_CAMERA_H

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include "opencv2/opencv.hpp"
#include <string>
#include <vector>
using namespace std;
using namespace cv;

class UndistortCamera
{
private:
  ros::NodeHandle nh;
  ros::Subscriber raw_image_sub;
  ros::Publisher undistorted_pub;

  // Create pointer to image
  // Class CvImage contains three variables:
  //  1. A std_msgs header
  //  2. A std::string encoding
  //  3. An image of type cv::Mat - this is the actual image object in c++
  cv_bridge::CvImagePtr cv_ptr;
  string camera_name, sub_topic, pub_topic, video_mode;
  double frame_rate;
  Size img_size;

  // Variables to undistort image
  Mat cameraMatrix, distortionCoeffs, map1, map2;

public:
  UndistortCamera();
  void LoadProperty(std::string name, double &param);
  void ImageCB(const sensor_msgs::ImageConstPtr& msg); // Image callback
  void Loop();
};

#endif
