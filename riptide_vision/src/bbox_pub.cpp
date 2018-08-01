#include "riptide_vision/bbox_pub.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_sim");
  BBoxPub ds;
  ros::spin();
}

BBoxPub::BBoxPub() : nh("darknet_sim")
{
  bbox_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1);
  bbox_sub = nh.subscribe<darknet_ros_msgs::BoundingBox>("/test/bbox_input", 1, &BBoxPub::BBoxCB, this);
}

void BBoxPub::BBoxCB(const darknet_ros_msgs::BoundingBox::ConstPtr &msg)
{
  darknet_ros_msgs::BoundingBoxes cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "detection";
  cmd.image_header.stamp = ros::Time::now();
  cmd.image_header.frame_id = "image";
  cmd.bounding_boxes.push_back(*msg);

  bbox_pub.publish(cmd);
}
