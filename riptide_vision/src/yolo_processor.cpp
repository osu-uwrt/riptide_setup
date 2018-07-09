#include "riptide_vision/yolo_processor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_processor");
  YoloProcessor yp;
  yp.Loop();
}

YoloProcessor::YoloProcessor() : nh("yolo_processor") {
  darknet_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &YoloProcessor::DarknetCB, this);
  task_sub = nh.subscribe<riptide_msgs::TaskInfo>("/task/info", 1, &YoloProcessor::TaskCB, this);

  task_bbox_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1);

  YoloProcessor::InitMsgs();
}

void YoloProcessor::InitMsgs() {

}

// Get darknet bboxes
void YoloProcessor::DarknetCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg) {

  task_bboxes.header.stamp = ros::Time::now();
  task_bboxes.header.frame_id = "task_bboxes";
  task_bbox_pub.publish(task_bboxes);
  //task_bboxes.bounding_boxes
}

// Get task info
void YoloProcessor::TaskCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg) {
  task_name = task_msg->task_name;
  alignment_plane = task_msg->alignment_plane;
  int i = 0;
  for(std::vector<int>::const_iterator it = task_msg->object_id.begin(); it != task_msg->object_id.end(); ++it) {
    object_id.push_back((int)*it);
    i++;
  }

  thresh = task_msg->thresh;
}

void YoloProcessor::Loop()
{
  ros::Rate rate(50);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
