#include "ros/ros.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/Constants.h"
#include "std_msgs/Float64.h"
#include <riptide_autonomy/AlignAction.h>
#include <actionlib/server/simple_action_server.h>

class AlignActionServer {
private:
  ros::NodeHandle nh;
  std::string action_name;
  actionlib::SimpleActionServer<riptide_autonomy::AlignAction> AlignServer;
  riptide_autonomy::AlignFeedback feedback;
  riptide_autonomy::AlignResult result;

  ros::Subscriber imu_sub, vision_sub, depth_sub;
  ros::Publisher accelX_pub, accelY_pub, accelZ_pub;

  riptide_msgs::Imu state_imu;
  riptide_msgs::Depth state_depth;
  riptide_msgs::Vision state_vision;

public:
  AlignActionServer(std::string name);
  ~AlignActionServer(void);
  void executeCB(const riptide_autonomy::AlignGoal::ConstPtr &goal);
  void imuCB(const riptide_msgs::Imu &imu);
  void depthCB(const riptide_msgs::Depth &depth);
  void visionCB(const riptide_msgs::Vision &vision);
}

AlignActionServer::AlignActionServer(std::string name) :
  AlignServer(nh, name, boost::bind(&AlignActionServer::executeCB, this, _1), false),
  action_name(name)
  {
    AlignServer.start();
    imu_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1, &AlignActionServer::imuCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1, &AlignActionServer::depthCB, this);
    accelX_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/x", 1);
    accelY_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/y", 1);
    accelZ_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/z", 1);
  }

AlignActionServer::~AlignActionServer() {}

AlignActionServer::executeCB(const riptide_autonomy::AlignGoal::ConstPtr &goal) {
  int RCOffset = goal->RCOffset;
  vision_sub = nh.subscribe<riptide_msgs::Vision>(vision_topics[goal->task - RCOffset], 1, &AlignActionServer::visionCB, this);
}

AlignActionServer::imuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  state_imu = imu_msg;
}

AlignActionServer::depthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {
  state_depth = depth_msg;
}

AlignActionServer::visionCB(const riptide_msgs::Vision::ConstPtr &vision_msg) {
  state_vision = vision_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "align_server");
  AlignActionServer align_server("align_server");
  ros::spin();
  return 0;
}
