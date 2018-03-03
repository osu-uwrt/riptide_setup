#include "ros/ros.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/Constants.h"
#include "std_msgs/Float64.h"
#include <riptide_autonomy/FindAction.h>
#include <actionlib/server/simple_action_server.h>

class FindActionServer {
private:
  ros::NodeHandle nh;
  std::string action_name;
  actionlib::SimpleActionServer<riptide_autonomy::FindAction> findServer;
  riptide_autonomy::FindFeedback feedback;
  riptide_autonomy::FindResult result;

  ros::Subscriber imu_sub, vision_sub, depth_sub;
  ros::Publisher accelX_pub, accelY_pub, accelZ_pub;

  riptide_msgs::Imu state_imu;
  riptide_msgs::Depth state_depth;
  riptide_msgs::Vision state_vision;

  std::string vision_topics[2];
  int topic;

public:
  FindActionServer(std::string name);
  ~FindActionServer(void);
  void executeCB(const riptide_autonomy::FindGoal::ConstPtr &goal);
  void imuCB(const riptide_msgs::Imu &imu);
  void depthCB(const riptide_msgs::Depth &depth);
  void genVisionCB(const riptide_msgs::Vision &vision);
}

FindActionServer::FindActionServer(std::string name) :
  findServer(nh, name, boost::bind(&FindActionServer::executeCB, this, _1), false),
  action_name(name)
  {
    findServer.start();
    imu_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1, &FindActionServer::imuCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1, &FindActionServer::depthCB, this);
    accelX_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/x", 1);
    accelY_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/y", 1);
    accelZ_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/z", 1);

    vision_topics = {"state/vision/qualify_gate/general",
                      "state/vision/qualify_marker/general"};
  }

FindActionServer::~FindActionServer() {}

FindActionServer::executeCB(const riptide_autonomy::FindGoal::ConstPtr &goal) {
  int RCOffset = goal->RCOffset;
  //vision_sub = nh.subscribe<riptide_msgs::Vision>(vision_topics[goal->task - RCOffset], 1, &FindActionServer::genVisionCB, this);
}

FindActionServer::imuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  state_imu.header = imu_msg->header;
  state_imu.euler_rpy = imu_msg->euler_rpy;
  state_imu.linear_acceleration = imu_msg->linear_acceleration;
  state_imu.angular_velocity = imu_msg->angular_velocity;
  state_imu.angular_acceleration = imu_msg->angular_acceleration;
}

FindActionServer::depthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {
  state_depth.header = depth_msg->header;
  state_depth.depth = depth_msg->depth;
  state_depth.pressure = depth_msg->pressure;
  state_depth.temp = depth_msg->temp;
  state_depth.altitude = depth_msg->altitude;
}

FindActionServer::genVisionCB(const riptide_msgs::GeneralVision::ConstPtr &gen_vis_msg) {
  //state_gen_vision = gen_vis_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "find_server");
  FindActionServer find_server("find_server");
  ros::spin();
  return 0;
}
