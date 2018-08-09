#ifndef EXTRACT_VIDEO_H
#define EXTRACT_VIDEO_H

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
//#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "std_msgs/Header.h"
#include "opencv2/opencv.hpp"
#include <string>
using namespace cv;
using namespace std;

/*extern "C" {
#include <libavutil/imgutils.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}*/

class ExtractVideo
{
private:
  ros::NodeHandle nh;
  ros::Subscriber image_sub;

  // Create pointer to image
  // Class CvImage contains three variables:
  //  1. A std_msgs header
  //  2. A std::string encoding
  //  3. An image of type cv::Mat - this is the actual image object in c++
  cv_bridge::CvImagePtr cv_ptr;

  VideoWriter videoWriter;
  string topic, file_name, ext, file_path, camera, username;
  int width, height, frame_rate, frames;

public:
  ExtractVideo();
  ~ExtractVideo();
  template <typename T>
  void LoadParam(string param, T &var);
  void WriteVideo(const sensor_msgs::ImageConstPtr& msg); // Image callback
  void Loop();
};

#endif
