#ifndef VALIDATOR_H
#define VALIDATOR_H

#include "ros/ros.h"

class DetectionValidator
{
private:
  double durationThresh;
  int detsReq;
  int detections;
  ros::Time startTime;

public:
  DetectionValidator(int detections, double duration);
  bool Validate();
};

class ErrorValidator
{
private:
  double durationThresh;
  double errorThresh;
  bool outsideRange;
  ros::Time startTime;

public:
  ErrorValidator(double errorThresh, double duration);
  bool Validate(double error);
  void Reset();
};

#endif