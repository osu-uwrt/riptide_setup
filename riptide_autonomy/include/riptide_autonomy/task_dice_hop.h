#ifndef DICE_HOP_H
#define DICE_HOP_H

#include "ros/ros.h"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "riptide_msgs/ControlStatusLinear.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/Pneumatics.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "riptide_autonomy/be_autonomous.h"
#include "riptide_autonomy/validators.h"

#include <cmath>
using namespace std;

class BeAutonomous;

class DiceHop
{
private:
  ros::Timer timer;

  // Reference to master
  BeAutonomous* master;

public:
  DiceHop(BeAutonomous* master);
  void Start();
  void DiceHopTimer(const ros::TimerEvent &event);
  void Abort();
};

#endif
