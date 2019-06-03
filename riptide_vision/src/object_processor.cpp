#include "riptide_vision/object_processor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_processor");
  ObjectProcessor yp;
  try
  {
    ros::spin();
  }
  catch (exception &e)
  {
    ROS_ERROR("Obj Pro Error: %s", e.what());
    ROS_ERROR("Obj Pro: Shutting Down");
  }
}

ObjectProcessor::ObjectProcessor() : nh("object_processor") {
  task_bbox_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &ObjectProcessor::TaskBBoxCB, this);
  image_sub = nh.subscribe<sensor_msgs::Image>("/stereo/left/image_undistorted", 1, &ObjectProcessor::ImageCB, this);
  task_info_sub = nh.subscribe<riptide_msgs::TaskInfo>("/task/info", 1, &ObjectProcessor::TaskInfoCB, this);
  alignment_cmd_sub = nh.subscribe<riptide_msgs::AlignmentCommand>("/command/alignment", 1, &ObjectProcessor::AlignmentCmdCB, this);

  object_pub = nh.advertise<riptide_msgs::Object>("/state/object", 1);
  object_image_pub = nh.advertise<sensor_msgs::Image>("/state/object_image", 1);

  colors.push_back(Scalar(255, 0, 0)); // Red
  colors.push_back(Scalar(0, 255, 0)); // Green
  colors.push_back(Scalar(255, 128, 0)); // Orange
  colors.push_back(Scalar(255, 0, 255)); // Purple

  // Initialize task info to Casino Gate
  task_file = rc::FILE_TASKS;
  task_id = 0;
  task_name = "Casino_Gate";
  object_name = "Casino_Gate_Black";
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
  ObjectProcessor::UpdateTaskInfo();

  current_attitude.x = 0;
  current_attitude.y = 0;
  current_attitude.z = 0;

  width = 644;
  height = 482;
  cam_center_x = width/2;
  cam_center_y = height/2;
}

// Load parameter from namespace
template <typename T>
void ObjectProcessor::LoadParam(string param, T &var)
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
    ROS_ERROR("Object Processor Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void ObjectProcessor::UpdateTaskInfo() {
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

void ObjectProcessor::ImageCB(const sensor_msgs::Image::ConstPtr &msg) {
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  width = cv_ptr->image.size().width;
  height = cv_ptr->image.size().height;
  cam_center_x = width/2;
  cam_center_y = height/2;

  // Uncomment when ready
  //sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", object_image).toImageMsg();
  //object_image_pub.publish(out_msg);
}

// Get task bboxes and locate bbox for desired object
void ObjectProcessor::TaskBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg) {
  bool found = false;

  // Search for bbox with object name
  for(int i=0; i<bbox_msg->bounding_boxes.size(); i++) {
    if(object_name == bbox_msg->bounding_boxes[i].Class) {
      found = true;
      object_bbox = bbox_msg->bounding_boxes[i];

      object.object_name = object_name;
      object.bbox_width = abs(object_bbox.xmax - object_bbox.xmin);
      object.bbox_height = abs(object_bbox.ymax - object_bbox.ymin);

      // Adjust positions so they are relative to the center of the CAMERA FRAME
      // Cam x-axis is pos. right; Cam y-axis is pos. down
      int xcenter = (object_bbox.xmin + object_bbox.xmax)/2 - cam_center_x;
      int ycenter = (object_bbox.ymin + object_bbox.ymax)/2 - cam_center_y;

      if(alignment_plane == riptide_msgs::Constants::PLANE_YZ) {
        object.pos.x = 0; // AUV x-axis is pos. fwd
        object.pos.y = -xcenter; // AUV y-axis is pos. port-side
        object.pos.z = -ycenter; // AUV z-axis is pos. up
      }
      else if(alignment_plane == riptide_msgs::Constants::PLANE_XY) {
        object.pos.x = -ycenter; // AUV x-axis is pos. fwd
        object.pos.y = -xcenter; // AUV y-axis is pos. port-side
        object.pos.z = 0; // AUV z-axis is pos. up
      }
      break;
    }
  }

  if(found) {
    object_pub.publish(object);
  }
}

// Get task info
void ObjectProcessor::TaskInfoCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg) {
  if(task_msg->task_id != task_id) {
    task_id = task_msg->task_id;
    ObjectProcessor::UpdateTaskInfo();

    // Subscribe to appropriate camera topic
    if(alignment_plane != last_alignment_plane) {
      image_sub.shutdown();
      image_sub = nh.subscribe<sensor_msgs::Image>(camera_topics[alignment_plane], 1, &ObjectProcessor::ImageCB, this);
    }
  }

  last_alignment_plane = alignment_plane;
}

void ObjectProcessor::AlignmentCmdCB(const riptide_msgs::AlignmentCommand::ConstPtr& cmd) {
  if(strcmp(cmd->object_name.c_str(), object_name.c_str()) != 0 ) // If name does not match, then update
    object_name = cmd->object_name;
}
