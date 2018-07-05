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
  std::string vision_topics[2];
  int topic;

public:
  AlignActionServer(std::string name);
  ~AlignActionServer(void);
  void executeCB(const riptide_autonomy::AlignGoal::ConstPtr &goal);
  void imuCB(const riptide_msgs::Imu &imu);
  void depthCB(const riptide_msgs::Depth &depth);
  void visionCB(const riptide_msgs::GeneralVision &gen_vis);
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

    vision_topics = {"state/vision/qualify_gate/general",
                      "state/vision/qualify_marker/general"};
  }

AlignActionServer::~AlignActionServer() {}

AlignActionServer::executeCB(const riptide_autonomy::AlignGoal::ConstPtr &goal) {
  int RCOffset = goal->RCOffset;
  vision_sub = nh.subscribe<riptide_msgs::Vision>(vision_topics[goal->task - RCOffset], 1, &AlignActionServer::visionCB, this);
  ros::Rate(1000);
}

AlignActionServer::imuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  state_imu.header = imu_msg->header;
  state_imu.euler_rpy = imu_msg->euler_rpy;
  state_imu.linear_acceleration = imu_msg->linear_acceleration;
  state_imu.angular_velocity = imu_msg->angular_velocity;
  state_imu.angular_acceleration = imu_msg->angular_acceleration;
}

AlignActionServer::depthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {
  state_depth.header = depth_msg->header;
  state_depth.depth = depth_msg->depth;
  state_depth.pressure = depth_msg->pressure;
  state_depth.temp = depth_msg->temp;
  state_depth.altitude = depth_msg->altitude;
}

AlignActionServer::visionCB(const riptide_msgs::GeneralVision::ConstPtr &gen_vis_msg) {
  //state_vision = vision_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "align_server");
  AlignActionServer align_server("align_server");
  ros::spin();
  return 0;
}
