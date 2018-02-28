#include "ros/ros.h"
#include "riptide_msgs/ImuVerbose.h"
#include <riptide_autonomy/AlignmentAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<riptide_autonomy::AlignmentAction> Server;

class AlignmentActionServer {
private:
  ros::NodeHandle nh;

public:
  void executeCB(const riptide_autonomy::AlignmentAction )
}
