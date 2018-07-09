#include "riptide_vision/yolo_proc_sim.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_proc_sim");
  YoloProcSim yps;
  yps.Loop();
}

YoloProcSim::YoloProcSim() : nh("yolo_proc_sim") {
  darknet_sim_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1);
  task_sim_pub = nh.advertise<riptide_msgs::TaskInfo>("/task/info", 1);

  YoloProcSim::LoadParam<string>("task_name", task_name);
  YoloProcSim::LoadParam<int>("alignment_plane", alignment_plane);
  YoloProcSim::LoadParam<double>("thresh", thresh);

  YoloProcSim::InitMsgs();
}

void YoloProcSim::InitMsgs() {

}

// Load parameter from namespace
template <typename T>
void YoloProcSim::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("Yolo Proc Sim Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

// Publish darknet bboxes
void YoloProcSim::DarknetPub() {
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

  bbox.Class = "Casino_Gate_Red";
  bbox.probability = 0.70;
  bbox.xmin = 325;
  bbox.ymin = 325;
  bbox.xmax = 600;
  bbox.ymax = 600;
  bboxes.bounding_boxes.push_back(bbox);

  bbox.Class = "Dice1";
  bbox.probability = 0.73;
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

  bbox.Class = "Dice5";
  bbox.probability = 0.90;
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

  bbox.Class = "Path_Marker";
  bbox.probability = 0.75;
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

  darknet_sim_pub.publish(bboxes);
  bboxes.bounding_boxes.clear();
}

// Publish task info
void YoloProcSim::TaskPub() {
  task_msg.task_name = task_name;
  task_msg.alignment_plane = alignment_plane;
  task_msg.thresh = thresh;

  if(strcmp(task_name.c_str(), "Casino_Gate") == 0) {
    task_msg.num_objects = 2;
    task_msg.object_id.push_back(0);
    task_msg.object_id.push_back(1);
  }
  else if(strcmp(task_name.c_str(), "Dice") == 0) {
    task_msg.num_objects = 4;
    task_msg.object_id.push_back(0);
    task_msg.object_id.push_back(1);
    task_msg.object_id.push_back(2);
    task_msg.object_id.push_back(3);
  }
  if(strcmp(task_name.c_str(), "Path_Marker") == 0) {
    task_msg.num_objects = 1;
    task_msg.object_id.push_back(0);
  }
  if(strcmp(task_name.c_str(), "Roulette") == 0) {
    task_msg.num_objects = 1;
    task_msg.object_id.push_back(0);
  }

  task_sim_pub.publish(task_msg);
}

void YoloProcSim::Loop()
{
  ros::Rate rate(15);
  while(ros::ok())
  {
    YoloProcSim::DarknetPub();
    YoloProcSim::TaskPub();
    ros::spinOnce();
    rate.sleep();
  }
}
