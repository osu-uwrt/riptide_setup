#ifndef CALIBRATE_CAMERA_H
#define CALIBRATE_CAMERA_H

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include "opencv2/opencv.hpp"
#include <string>
#include <vector>
using namespace std;
using namespace cv;

class CalibrateCamera
{
private:
  ros::NodeHandle nh;
  ros::Subscriber raw_image_sub;

  // Create pointer to image
  // Class CvImage contains three variables:
  //  1. A std_msgs header
  //  2. A std::string encoding
  //  3. An image of type cv::Mat - this is the actual image object in c++
  cv_bridge::CvImagePtr cv_ptr;
  string camera_name, sub_topic;
  bool calculated;

  int numBoards, numCornersHor, numCornersVer, numSquares, successes, pauses;
  double frame_rate;
  bool done;

  // Variables to collect data
  Mat gray_image;
  Size board_sz;
  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > image_points;
  vector<Point2f> corners;
  vector<Point3f> obj;

  // Variables to calculate calibraton
  Mat cameraMatrix, distortionCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;

public:
  CalibrateCamera();
  void ImageCB(const sensor_msgs::ImageConstPtr& msg); // Image callback
  void Loop();
};

#endif
