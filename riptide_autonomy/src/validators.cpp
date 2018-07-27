#include "riptide_autonomy/validators.h"

DetectionValidator::DetectionValidator(int detections, double duration)
{
  durationThresh = duration;
  detsReq = detections;
}

bool DetectionValidator::Validate()
{
  if (++detections == 1)
    startTime = ros::Time::now();

  if (ros::Time::now().toSec() - startTime.toSec() > durationThresh)
  {
    bool valid = detections >= detsReq;
    detections = 0;
    return valid;
  }
}

ErrorValidator::ErrorValidator(double errorThresh, double duration)
{
  durationThresh = duration;
  this->errorThresh = errorThresh;
  outsideRange = true;
}

bool ErrorValidator::Validate(double error)
{
  if (abs(error) <= errorThresh)
  {
    if (outsideRange)
      startTime = ros::Time::now();

    outsideRange = false;

    if (ros::Time::now().toSec() - startTime.toSec() > durationThresh)
      return true;
  }
  else
    outsideRange = true;

  return false;
}

void ErrorValidator::Reset()
{
  outsideRange = true;
}