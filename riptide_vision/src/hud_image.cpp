#include "riptide_vision/hud_image.h"

#define GRAVITY 9.81 // [m/s^2]
#define WATER_DENSITY 1000 // [kg/m^3]

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hud_image");
  HUDImage hu;
  hu.Loop();
}

HUDImage::HUDImage() : nh("hud_image") {
  fwd_img_sub = nh.subscribe<sensor_msgs::Image>("/forward/image_undistorted", 1, &HUDImage::ForwardImgCB, this);
  down_img_sub = nh.subscribe<sensor_msgs::Image>("/downward/image_undistorted", 1, &HUDImage::DownwardImgCB, this);
  darknet_img_sub = nh.subscribe<sensor_msgs::Image>("/darknet_ros/detection_image", 1, &HUDImage::DarknetImgCB, this);
  imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &HUDImage::ImuCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &HUDImage::DepthCB, this);

  cmd_attitude_sub = nh.subscribe<geometry_msgs::Vector3>("/command/manual/attitude", 1, &HUDImage::CmdAttitudeCB, this);
  cmd_depth_sub = nh.subscribe<riptide_msgs::DepthCommand>("/command/manual/depth", 1, &HUDImage::CmdDepthCB, this);
  cmd_accel_sub = nh.subscribe<geometry_msgs::Accel>("/command/accel", 1, &HUDImage::CmdAccelCB, this);

  fwd_img_pub = nh.advertise<sensor_msgs::Image>("/forward/image_hud", 1);
  down_img_pub = nh.advertise<sensor_msgs::Image>("/downward/image_hud", 1);
  darknet_img_pub = nh.advertise<sensor_msgs::Image>("/darknet_ros/image_hud", 1);

  topMargin = 80;
  botMargin = 0;

  HUDImage::InitMsgs();
}

void HUDImage::InitMsgs() {
  euler_rpy.x = 0;
  euler_rpy.y = 0;
  euler_rpy.z = 0;
  linear_accel.x = 0;
  linear_accel.y = 0;
  linear_accel.z = 0;
  depth = 0;

  cmd_euler_rpy.x = 0;
  cmd_euler_rpy.y = 0;
  cmd_euler_rpy.z = 0;
  cmd_linear_accel.x = 0;
  cmd_linear_accel.y = 0;
  cmd_linear_accel.z = 0;
  cmd_depth = 0;
}

void HUDImage::ForwardImgCB(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  copyMakeBorder(cv_ptr->image, cv_ptr->image, topMargin, botMargin, 0, 0, BORDER_CONSTANT, Scalar(255,255,255));
  string rpyd = sprintf("Roll: %f, Pitch: %f, Yaw: %f, Depth: %f", euler_rpy.x, euler_rpy.y, euler_rpy.z, depth);

  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
  fwd_img_pub.publish(out_msg);
}

void HUDImage::DownwardImgCB(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
  down_img_pub.publish(out_msg);
}

void HUDImage::DarknetImgCB(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
  darknet_img_pub.publish(out_msg);
}

// Get current orientation and linear accel
void HUDImage::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  euler_rpy.x = imu_msg->euler_rpy.x;
  euler_rpy.y = imu_msg->euler_rpy.y;
  euler_rpy.z = imu_msg->euler_rpy.z;

  linear_accel.x = imu_msg->linear_accel.x;
  linear_accel.y = imu_msg->linear_accel.y;
  linear_accel.z = imu_msg->linear_accel.z;
}

// Get current depth
void HUDImage::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {
  depth = depth_msg->depth;
}

// Get command attitude
void HUDImage::CmdAttitudeCB(const geometry_msgs::Vector3::ConstPtr& cmd_msg) {
  cmd_euler_rpy.x = cmd_msg->x;
  cmd_euler_rpy.y = cmd_msg->y;
  cmd_euler_rpy.z = cmd_msg->z;
}

// Get command depth
void HUDImage::CmdDepthCB(const riptide_msgs::DepthCommand::ConstPtr& cmd_msg) {
  cmd_depth = cmd_msg->absolute;
}

// Get command linear accel
void HUDImage::CmdAccelCB(const geometry_msgs::Accel::ConstPtr& cmd_msg){
  cmd_linear_accel.x = cmd_msg->linear.x;
  cmd_linear_accel.y = cmd_msg->linear.y;
  cmd_linear_accel.z = cmd_msg->linear.z;
}

void HUDImage::Loop()
{
  ros::Rate rate(200);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
