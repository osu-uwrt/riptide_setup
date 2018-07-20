#ifndef OBJECT_DESCRIBER_H
#define OBJECT_DESCRIBER_H

#include "ros/ros.h"
#include <cmath>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "be_autonomous.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

#define IMAGES_TO_SAMPLE 5

class BeAutonomous;
class Roulette;

class ObjectDescriber
{

private:
  ros::Subscriber bbox_sub, image_sub;
  BeAutonomous* master;

  Mat lastImage;

  void (Roulette::*rouletteCallbackFunction)(double);
  Roulette *rouletteCallbackObject;
  double averageRouletteAngle = 0;
  int rouletteImagesLeft = 0;

public:
  ObjectDescriber(BeAutonomous* master);

  void GetRouletteHeading(void (Roulette::*callback)(double), Roulette *object);
  void BBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox);
  void ImageCB(const sensor_msgs::Image::ConstPtr& image);
};

#endif
