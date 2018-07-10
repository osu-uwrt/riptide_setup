#include "riptide_vision/darknet_sim.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "darknet_sim");
  DarknetSim ds;
  ds.Loop();
}

DarknetSim::DarknetSim() : nh("darknet_sim") {
  darknet_sim_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1);
}

// Publish darknet bboxes
void DarknetSim::DarknetPub() {
  bboxes.header.stamp = ros::Time::now();
  bboxes.header.frame_id = "detection";
  bboxes.image_header.stamp = ros::Time::now();
  bboxes.image_header.frame_id = "image";

  // Make fake bboxes
  darknet_ros_msgs::BoundingBox bbox;
  bbox.Class = "Casino_Gate_Black";
  bbox.probability = 0.6;
  bbox.xmin = 25;
  bbox.ymin = 25;
  bbox.xmax = 325;
  bbox.ymax = 325;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Casino_Gate_Black";
  bbox.probability = 0.2;
  bbox.xmin = 5;
  bbox.ymin = 5;
  bbox.xmax = 325;
  bbox.ymax = 325;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Casino_Gate_Red";
  bbox.probability = 0.70;
  bbox.xmin = 325;
  bbox.ymin = 325;
  bbox.xmax = 600;
  bbox.ymax = 600;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Casino_Gate_Red";
  bbox.probability = 0.150;
  bbox.xmin = 225;
  bbox.ymin = 225;
  bbox.xmax = 500;
  bbox.ymax = 500;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice1";
  bbox.probability = 0.73;
  bbox.xmin = 70;
  bbox.ymin = 300;
  bbox.xmax = 120;
  bbox.ymax = 350;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice1";
  bbox.probability = 0.57;
  bbox.xmin = 70;
  bbox.ymin = 300;
  bbox.xmax = 120;
  bbox.ymax = 350;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice2";
  bbox.probability = 0.95;
  bbox.xmin = 50;
  bbox.ymin = 50;
  bbox.xmax = 100;
  bbox.ymax = 100;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice2";
  bbox.probability = 0.77;
  bbox.xmin = 50;
  bbox.ymin = 50;
  bbox.xmax = 100;
  bbox.ymax = 100;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice5";
  bbox.probability = 0.90;
  bbox.xmin = 75;
  bbox.ymin = 75;
  bbox.xmax = 150;
  bbox.ymax = 150;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice5";
  bbox.probability = 0.84;
  bbox.xmin = 75;
  bbox.ymin = 75;
  bbox.xmax = 150;
  bbox.ymax = 150;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice6";
  bbox.probability = 0.97;
  bbox.xmin = 100;
  bbox.ymin = 100;
  bbox.xmax = 150;
  bbox.ymax = 150;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice6";
  bbox.probability = 0.86;
  bbox.xmin = 100;
  bbox.ymin = 100;
  bbox.xmax = 150;
  bbox.ymax = 150;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Path_Marker";
  bbox.probability = 0.75;
  bbox.xmin = 50;
  bbox.ymin = 50;
  bbox.xmax = 150;
  bbox.ymax = 150;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Path_Marker";
  bbox.probability = 0.43;
  bbox.xmin = 50;
  bbox.ymin = 50;
  bbox.xmax = 150;
  bbox.ymax = 150;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Roulette";
  bbox.probability = 0.99;
  bbox.xmin = 100;
  bbox.ymin = 100;
  bbox.xmax = 500;
  bbox.ymax = 500;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Roulette";
  bbox.probability = 0.69;
  bbox.xmin = 100;
  bbox.ymin = 100;
  bbox.xmax = 500;
  bbox.ymax = 500;
  bboxes.bounding_boxes.push_back(bbox);

  darknet_sim_pub.publish(bboxes);
  bboxes.bounding_boxes.clear();
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
