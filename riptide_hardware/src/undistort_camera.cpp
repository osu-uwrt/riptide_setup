#include "riptide_hardware/undistort_camera.h"
//using namespace cv;
#define MAX_WIDTH 1288
#define MAX_HEIGHT 964

int main(int argc, char** argv) {
  ros::init(argc, argv, "undistort_camera");
  UndistortCamera uc;
  uc.Loop();
}

UndistortCamera::UndistortCamera() : nh("undistort_camera") {
  // Initialize size and type for Mat objects
  // NOTE: Mat type MUST be CV_64FC1 (cannot be CV_32FC1)
  cameraMatrix = Mat(3, 3, CV_64FC1);
  distortionCoeffs = Mat(1, 5, CV_64FC1);

  nh.getParam("camera_name", camera_name);
  UndistortCamera::LoadProperty("frame_rate", frame_rate);
  nh.getParam("video_mode", video_mode);

  // Populate Mat objects from camera calibration file
  UndistortCamera::LoadProperty("cameraMatrix/R0/C0", cameraMatrix.ptr<double>(0)[0]);
  UndistortCamera::LoadProperty("cameraMatrix/R0/C1", cameraMatrix.ptr<double>(0)[1]);
  UndistortCamera::LoadProperty("cameraMatrix/R0/C2", cameraMatrix.ptr<double>(0)[2]);
  UndistortCamera::LoadProperty("cameraMatrix/R1/C0", cameraMatrix.ptr<double>(1)[0]);
  UndistortCamera::LoadProperty("cameraMatrix/R1/C1", cameraMatrix.ptr<double>(1)[1]);
  UndistortCamera::LoadProperty("cameraMatrix/R1/C2", cameraMatrix.ptr<double>(1)[2]);
  UndistortCamera::LoadProperty("cameraMatrix/R2/C0", cameraMatrix.ptr<double>(2)[0]);
  UndistortCamera::LoadProperty("cameraMatrix/R2/C1", cameraMatrix.ptr<double>(2)[1]);
  UndistortCamera::LoadProperty("cameraMatrix/R2/C2", cameraMatrix.ptr<double>(2)[2]);

  UndistortCamera::LoadProperty("distortionCoeffs/C0", distortionCoeffs.ptr<double>(0)[0]);
  UndistortCamera::LoadProperty("distortionCoeffs/C1", distortionCoeffs.ptr<double>(0)[1]);
  UndistortCamera::LoadProperty("distortionCoeffs/C2", distortionCoeffs.ptr<double>(0)[2]);
  UndistortCamera::LoadProperty("distortionCoeffs/C3", distortionCoeffs.ptr<double>(0)[3]);
  UndistortCamera::LoadProperty("distortionCoeffs/C4", distortionCoeffs.ptr<double>(0)[4]);

  // Display to screen to verify config parameters read properly
  ROS_INFO("Intrincic [0][0] %f", cameraMatrix.ptr<double>(0)[0]);
  ROS_INFO("cameraMatrix [0][1] %f", cameraMatrix.ptr<double>(0)[1]);
  ROS_INFO("cameraMatrix [0][2] %f", cameraMatrix.ptr<double>(0)[2]);
  ROS_INFO("cameraMatrix [1][0] %f", cameraMatrix.ptr<double>(1)[0]);
  ROS_INFO("cameraMatrix [1][1] %f", cameraMatrix.ptr<double>(1)[1]);
  ROS_INFO("cameraMatrix [1][2] %f", cameraMatrix.ptr<double>(1)[2]);
  ROS_INFO("cameraMatrix [2][0] %f", cameraMatrix.ptr<double>(2)[0]);
  ROS_INFO("cameraMatrix [2][1] %f", cameraMatrix.ptr<double>(2)[1]);
  ROS_INFO("cameraMatrix [2][2] %f", cameraMatrix.ptr<double>(2)[2]);

  ROS_INFO("Dist. coef [0][0] %f", distortionCoeffs.ptr<double>(0)[0]);
  ROS_INFO("Dist. coef [0][1] %f", distortionCoeffs.ptr<double>(0)[1]);
  ROS_INFO("Dist. coef [0][2] %f", distortionCoeffs.ptr<double>(0)[2]);
  ROS_INFO("Dist. coef [0][3] %f", distortionCoeffs.ptr<double>(0)[3]);
  ROS_INFO("Dist. coef [0][4] %f", distortionCoeffs.ptr<double>(0)[4]);

  // Create subscriber and publisher topics
  sub_topic = "/" + camera_name + "/image_raw";
  pub_topic = "/" + camera_name + "/image_undistorted";

  raw_image_sub = nh.subscribe<sensor_msgs::Image>(sub_topic, 1, &UndistortCamera::ImageCB, this);
  undistorted_pub = nh.advertise<sensor_msgs::Image>(pub_topic, 1);

  // Get image size based on video mode chosen
  if(strcmp(video_mode.c_str(), "format7_mode0") == 0)
    img_size = Size(MAX_WIDTH, MAX_HEIGHT);
  else if(strcmp(video_mode.c_str(), "format7_mode1") == 0)
    img_size = Size(MAX_WIDTH/2, MAX_HEIGHT/2);
  else {
    ROS_ERROR("Video mode must be one of 'format7_mode0' or 'format7_mode1'");
    ros::shutdown();
  }

  // Speed up undistort process by calculting the map1 and map2 variables once:
  // This function is called within the cv::undistort function, yet it takes about 90% of the computation time (about 90 ms).
  // Just do it once and speed up the process by a lot!
  initUndistortRectifyMap(cameraMatrix, distortionCoeffs, Mat(), cameraMatrix, img_size, CV_16SC2, map1, map2);
}

// Load property from namespace
void UndistortCamera::LoadProperty(std::string name, double &param)
{
  try
  {
    if (!nh.getParam(name, param))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    ROS_ERROR("Critical! Undistort Camera (camera %s) has no property set for %s. Shutting down...", camera_name.c_str(), name.c_str());
    ros::shutdown();
  }
}

void UndistortCamera::ImageCB(const sensor_msgs::Image::ConstPtr &msg) {
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  Mat imageUndistorted;
  // This replaces the second function call in cv::undistort(). Only takes about 10 ms
  remap(cv_ptr->image, imageUndistorted, map1, map2, INTER_LINEAR);

  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageUndistorted).toImageMsg();
  undistorted_pub.publish(out_msg);

  cv_ptr.reset();
}

void UndistortCamera::Loop()
{
  ros::Rate rate(frame_rate);
  while(!ros::isShuttingDown()) {
    ros::spinOnce();
    rate.sleep();
  }
}
