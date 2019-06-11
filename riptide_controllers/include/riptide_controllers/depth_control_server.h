#ifndef DEPTH_CONTROL_SERVER_H
#define DEPTH_CONTROL_SERVER_H

#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "riptide_msgs/DepthCommand.h"
using namespace std;

class DepthControlActionServer
{
  private:
    // Comms
    ros::NodeHandle nh;
    
    ros::Publisher depth_pub;

    

    

  public:
    DepthControlActionServer();
    DepthControlActionServer(std::string name);
    void executeCB(const riptide_controllers::depth_commandGoalConstPtr &goal);
    
    
 };

 #endif
