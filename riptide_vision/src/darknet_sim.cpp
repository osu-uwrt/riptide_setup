#include "riptide_vision/darknet_sim.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "darknet_sim");
  DarknetSim ds;
  ds.Loop();
}

DarknetSim::DarknetSim() : nh("darknet_sim") {
  darknet_sim_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1);

  tasks_sim_file = rc::FILE_TASKS_SIM;
  tasks = YAML::LoadFile(tasks_sim_file);
  DarknetSim::LoadSimData();
}

void DarknetSim::LoadSimData() {
  darknet_ros_msgs::BoundingBox bbox;
  for(int i=0; i<(int)tasks["tasks"].size(); i++) {
    bbox.Class = tasks["tasks"][i]["class"].as<string>();
    bbox.probability = tasks["tasks"][i]["probability"].as<double>();
    bbox.xmin = tasks["tasks"][i]["xmin"].as<int>();
    bbox.ymin = tasks["tasks"][i]["ymin"].as<int>();
    bbox.xmax = tasks["tasks"][i]["xmax"].as<int>();
    bbox.ymax = tasks["tasks"][i]["ymax"].as<int>();
    bboxes.bounding_boxes.push_back(bbox);
  }
}

// Publish darknet bboxes
void DarknetSim::DarknetPub() {
  bboxes.header.stamp = ros::Time::now();
  bboxes.header.frame_id = "detection";
  bboxes.image_header.stamp = ros::Time::now();
  bboxes.image_header.frame_id = "image";

  darknet_sim_pub.publish(bboxes);
}

void DarknetSim::Loop()
{
  ros::Rate rate(15);
  while(ros::ok())
  {
    DarknetSim::DarknetPub();
    ros::spinOnce();
    rate.sleep();
  }
}
