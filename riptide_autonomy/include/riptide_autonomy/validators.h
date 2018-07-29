#ifndef VALIDATOR_H
#define VALIDATOR_H

#include "ros/ros.h"

class DetectionValidator
{
private:
  double durationThresh;
  int detsReq;
  int detections;
  int attempts;
  ros::Time startTime;
  bool valid;

public:
  DetectionValidator(int detections, double duration);
  bool Validate();
  bool IsValid();
  int GetDetections();
  int GetAttempts();
  void Reset();
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
  bool IsValid();
  int GetDetections();
  int GetAttempts();
  void Reset();
};

#endif
