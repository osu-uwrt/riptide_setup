#ifndef CALIBRATE_CAMERA_H
#define CALIBRATE_CAMERA_H

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "std_msgs/Header.h"
#include "opencv2/opencv.hpp"
#include <string>
#include <vector>
using namespace std;
using namespace cv;

/*template<typename _Tp>
class cv::Point3_< _Tp>
class cv::Point2_< _Tp>

typedef Point3_<float> Point3f;
typedef Point2_<float> Point2f;*/

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
  std::string camera, sub_topic, pub_topic;
  bool calculated;

  int numBoards, numCornersHor, numCornersVer, numSquares, successes, pauses;
  double frame_rate;
  bool done;

  // Variables to collect data
  cv::Mat gray_image;
  cv::Size board_sz;
  vector<vector<cv::Point3f> > object_points;
  vector<vector<cv::Point2f> > image_points;
  vector<cv::Point2f> corners;
  vector<cv::Point3f> obj;

  // Variables to calculate calibraton
  cv::Mat intrinsic;
  cv::Mat distCoeffs;
  vector<cv::Mat> rvecs;
  vector<cv::Mat> tvecs;

public:
  CalibrateCamera();
  void ImageCB(const sensor_msgs::ImageConstPtr& msg); // Image callback
  void Loop();
};

#endif
