#include "riptide_vision/yolo_processor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_processor");
  YoloProcessor yp;
  ros::spin();
}

YoloProcessor::YoloProcessor() : nh("yolo_processor") {
  darknet_bbox_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &YoloProcessor::DarknetBBoxCB, this);
  image_sub = nh.subscribe<sensor_msgs::Image>("/forward/image_undistorted", 1, &YoloProcessor::ImageCB, this);
  task_info_sub = nh.subscribe<riptide_msgs::TaskInfo>("/task/info", 1, &YoloProcessor::TaskInfoCB, this);

  task_bbox_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1);
  task_image_pub = nh.advertise<sensor_msgs::Image>("/task/detection_image", 1);
  low_detections_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/task/low_detections", 1);

  colors.push_back(Scalar(255, 0, 0)); // Red
  colors.push_back(Scalar(0, 255, 0)); // Green
  colors.push_back(Scalar(255, 128, 0)); // Orange
  colors.push_back(Scalar(255, 0, 255)); // Purple
  margin_color = Scalar(255, 255, 255); // White

  top_margin = 120;
  num_rows = 4;
  offset = top_margin/15;
  for(int i = 0; i < num_rows; i++)
    text_start[i] = (i+1.0)/num_rows*top_margin - offset;

  // Initialize task info to Casino gate
  task_id = 0;
  task_file = rc::FILE_TASKS;
  task_name = "Casino_Gate";
  alignment_plane = rc::PLANE_YZ;

  tasks = YAML::LoadFile(task_file);

  // Verify number of objects and thresholds match
  num_tasks = (int)tasks["tasks"].size();
  for(int i=0; i<num_tasks; i++) {
    num_objects = (int)tasks["tasks"][i]["objects"].size();
    num_thresholds = (int)tasks["tasks"][i]["thresholds"].size();
    task_name = tasks["tasks"][i]["name"].as<string>();
    if(num_objects != num_thresholds) {
      ROS_INFO("Task ID %i (%s): %i objects and %i thresholds. Quantities must match", i, task_name.c_str(), num_objects, num_thresholds);
      ROS_INFO("Shutting Down");
      ros::shutdown();
    }
  }

  // Update task info
  YoloProcessor::UpdateTaskInfo();
}

// Load parameter from namespace
template <typename T>
void YoloProcessor::LoadParam(string param, T &var)
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
    ROS_ERROR("Yolo Processor Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void YoloProcessor::UpdateTaskInfo() {
  task_name = tasks["tasks"][task_id]["name"].as<string>();
  num_objects = (int)tasks["tasks"][task_id]["objects"].size();

  alignment_plane = tasks["tasks"][task_id]["plane"].as<int>();
  if(alignment_plane != rc::PLANE_YZ && alignment_plane != rc::PLANE_XY)
    alignment_plane = rc::PLANE_YZ; // Default to YZ-plane (fwd cam)

  object_names.clear();
  thresholds.clear();
  for(int i=0; i < num_objects; i++) {
    object_names.push_back(tasks["tasks"][task_id]["objects"][i].as<string>());
    thresholds.push_back(tasks["tasks"][task_id]["thresholds"][i].as<double>());
  }
}

void YoloProcessor::ImageCB(const sensor_msgs::Image::ConstPtr &msg) {
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  // Create and publish task detection image
  copyMakeBorder(cv_ptr->image, task_image, top_margin, 0, 0, 0, BORDER_CONSTANT, margin_color);
  int thickness = 2;
  double font_scale = 1;

  // Should only have at most 4 bboxes (only Dice/Slots have 4 classes)
  for(int i=0; i<task_bboxes.bounding_boxes.size(); i++) {
    darknet_ros_msgs::BoundingBox bbox = task_bboxes.bounding_boxes[i];
    rectangle(task_image, Point(bbox.xmin, bbox.ymin + top_margin), Point(bbox.xmax, bbox.ymax + top_margin), colors.at(i), thickness);
    char text[100];
    sprintf(text, "%s: %.5f%%", bbox.Class.c_str(), bbox.probability);
    putText(task_image, string(text), Point(5, text_start[i]), FONT_HERSHEY_COMPLEX_SMALL, font_scale, colors.at(i), thickness);
  }

  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", task_image).toImageMsg();
  task_image_pub.publish(out_msg);
}

// Get darknet bboxes for desired task
void YoloProcessor::DarknetBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg) {
  // Create headers for bbox output
  task_bboxes.header.stamp = ros::Time::now();
  task_bboxes.header.frame_id = "task_bboxes";
  task_bboxes.image_header.stamp = ros::Time::now();
  task_bboxes.image_header.frame_id = "yolo_processed_image";

  low_detections.header.stamp = ros::Time::now();
  low_detections.header.frame_id = "low_detection_bboxes";
  low_detections.image_header.stamp = ros::Time::now();
  low_detections.image_header.frame_id = "yolo_processed_image";

  task_bboxes.bounding_boxes.clear();
  low_detections.bounding_boxes.clear();

  // Extract task bboxes witin specified threshold
  // Only append objects with highest probability (if multiple instances exist)
  // If any objects are below the threshold, then append to low_detections
  for(int i=0; i<num_objects; i++) {
    double max_prob = 0;
    darknet_ros_msgs::BoundingBox max_detection;

    for(int j=0; j<bbox_msg->bounding_boxes.size(); j++) {
      if(strcmp(object_names.at(i).c_str(), bbox_msg->bounding_boxes[j].Class.c_str()) == 0 ) {
        if(bbox_msg->bounding_boxes[j].probability > max_prob) {
          max_prob = bbox_msg->bounding_boxes[j].probability;
          max_detection = bbox_msg->bounding_boxes[j];
        }
      }
    }
    if(max_prob > thresholds.at(i)) {
      task_bboxes.bounding_boxes.push_back(max_detection);
    }
    else if(max_prob > 0) {
      low_detections.bounding_boxes.push_back(max_detection);
    }
  }

  if(task_bboxes.bounding_boxes.size() > 0)
    task_bbox_pub.publish(task_bboxes);
  if(low_detections.bounding_boxes.size() > 0)
    low_detections_pub.publish(low_detections);
}

// Get task info
void YoloProcessor::TaskInfoCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg) {
  if(task_msg->task_id != task_id) {
    task_id = task_msg->task_id;
    YoloProcessor::UpdateTaskInfo();

    // Subscribe to appropriate camera topic
    if(alignment_plane != last_alignment_plane) {
      image_sub.shutdown();
      image_sub = nh.subscribe<sensor_msgs::Image>(camera_topics[alignment_plane], 1, &YoloProcessor::ImageCB, this);
    }
  }

  last_alignment_plane = alignment_plane;
}
