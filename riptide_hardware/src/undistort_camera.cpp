#include "riptide_hardware/undistort_camera.h"
//using namespace cv;
#define MAX_WIDTH 1288
#define MAX_HEIGHT 964

int main(int argc, char** argv) {
  ros::init(argc, argv, "undistort_camera");
  UndistortCamera uc;
  uc.Loop();
}

UndistortCamera::UndistortCamera() : nh("undistort_camera") { // NOTE: there is no namespace declared in nh()
  // Initialize size and type for Mat objects
  // NOTE: Mat type MUST be CV_64FC1 (cannot be CV_32FC1)
  cameraMatrix = Mat(3, 3, CV_64FC1);
  distortionCoeffs = Mat(1, 5, CV_64FC1);

  //nh.getParam("camera_name", camera_name);
  UndistortCamera::LoadParam<string>("camera_name", camera_name);
  UndistortCamera::LoadParam<double>("frame_rate", frame_rate);
  //nh.getParam("video_mode", video_mode);
  UndistortCamera::LoadParam<string>("video_mode", video_mode);

  // Get image size based on video mode chosen, and set scale factor
  double scale_factor;
  if(video_mode == "format7_mode0") {
    img_size = Size(MAX_WIDTH, MAX_HEIGHT);
    scale_factor = 1.0;
  }
  else if(video_mode == "format7_mode1") {
    img_size = Size(MAX_WIDTH/2, MAX_HEIGHT/2);
    scale_factor = 0.5;
  }
  else {
    ROS_ERROR("Video mode must be one of 'format7_mode0' or 'format7_mode1'");
    ros::shutdown();
  }

  // Populate Mat objects from camera calibration file
  UndistortCamera::LoadParam<double>("cameraMatrix/R0/C0", cameraMatrix.ptr<double>(0)[0]);
  UndistortCamera::LoadParam<double>("cameraMatrix/R0/C1", cameraMatrix.ptr<double>(0)[1]);
  UndistortCamera::LoadParam<double>("cameraMatrix/R0/C2", cameraMatrix.ptr<double>(0)[2]);
  UndistortCamera::LoadParam<double>("cameraMatrix/R1/C0", cameraMatrix.ptr<double>(1)[0]);
  UndistortCamera::LoadParam<double>("cameraMatrix/R1/C1", cameraMatrix.ptr<double>(1)[1]);
  UndistortCamera::LoadParam<double>("cameraMatrix/R1/C2", cameraMatrix.ptr<double>(1)[2]);
  UndistortCamera::LoadParam<double>("cameraMatrix/R2/C0", cameraMatrix.ptr<double>(2)[0]);
  UndistortCamera::LoadParam<double>("cameraMatrix/R2/C1", cameraMatrix.ptr<double>(2)[1]);
  UndistortCamera::LoadParam<double>("cameraMatrix/R2/C2", cameraMatrix.ptr<double>(2)[2]);

  UndistortCamera::LoadParam<double>("distortionCoeffs/C0", distortionCoeffs.ptr<double>(0)[0]);
  UndistortCamera::LoadParam<double>("distortionCoeffs/C1", distortionCoeffs.ptr<double>(0)[1]);
  UndistortCamera::LoadParam<double>("distortionCoeffs/C2", distortionCoeffs.ptr<double>(0)[2]);
  UndistortCamera::LoadParam<double>("distortionCoeffs/C3", distortionCoeffs.ptr<double>(0)[3]);
  UndistortCamera::LoadParam<double>("distortionCoeffs/C4", distortionCoeffs.ptr<double>(0)[4]);

  /*ROS_INFO("\tBefore values changed. Camera Name: %s", camera_name.c_str());
  ROS_INFO("\tAddress cameraMatrix [0][0] %d", cameraMatrix.at<uchar>(0));
  ROS_INFO("\tAddress cameraMatrix [0][1] %d", cameraMatrix.at<uchar>(1));
  ROS_INFO("\tAddress cameraMatrix [0][2] %d", cameraMatrix.at<uchar>(2));*/

  // Scale fx, fy, cx, and cy in cameraMatrix by scale factor (based on video mode)
  /*double* p = cameraMatrix.ptr<double>(0); // Get pointer to first row
  p[0] = p[0] * scale_factor;
  p[1] = p[1] * scale_factor;
  p[2] = p[2] * scale_factor;
  p[3] = p[3] * scale_factor; // Can keep going b/c this mat object is continuous, so treat matrix like large 1-D array
  p[4] = p[4] * scale_factor;
  p[5] = p[5] * scale_factor;*/
  cameraMatrix.ptr<double>(0)[0] *= scale_factor;
  cameraMatrix.ptr<double>(0)[1] *= scale_factor;
  cameraMatrix.ptr<double>(0)[2] *= scale_factor;
  cameraMatrix.ptr<double>(0)[3] *= scale_factor;
  cameraMatrix.ptr<double>(0)[4] *= scale_factor;
  cameraMatrix.ptr<double>(0)[5] *= scale_factor;

  // Display to screen to verify config parameters read properly
  string t = "true", f = "false"; // Use with '<expression>?a:b' --> if expression is 'true' return a, else return b
  ROS_INFO("Camera Name: %s", camera_name.c_str());
  ROS_INFO("\tIs cameraMatrix continuous: %s", cameraMatrix.isContinuous()?t.c_str():f.c_str());
  ROS_INFO("\tIs distortionCoeffs continuous: %s", distortionCoeffs.isContinuous()?t.c_str():f.c_str());

  ROS_INFO("\tcameraMatrix [0][0] %f", cameraMatrix.ptr<double>(0)[0]);
  ROS_INFO("\tcameraMatrix [0][1] %f", cameraMatrix.ptr<double>(0)[1]);
  ROS_INFO("\tcameraMatrix [0][2] %f", cameraMatrix.ptr<double>(0)[2]);
  ROS_INFO("\tcameraMatrix [1][0] %f", cameraMatrix.ptr<double>(1)[0]);
  ROS_INFO("\tcameraMatrix [1][1] %f", cameraMatrix.ptr<double>(1)[1]);
  ROS_INFO("\tcameraMatrix [1][2] %f", cameraMatrix.ptr<double>(1)[2]);
  ROS_INFO("\tcameraMatrix [2][0] %f", cameraMatrix.ptr<double>(2)[0]);
  ROS_INFO("\tcameraMatrix [2][1] %f", cameraMatrix.ptr<double>(2)[1]);
  ROS_INFO("\tcameraMatrix [2][2] %f", cameraMatrix.ptr<double>(2)[2]);

  ROS_INFO("\tDist. coef [0][0] %f", distortionCoeffs.ptr<double>(0)[0]);
  ROS_INFO("\tDist. coef [0][1] %f", distortionCoeffs.ptr<double>(0)[1]);
  ROS_INFO("\tDist. coef [0][2] %f", distortionCoeffs.ptr<double>(0)[2]);
  ROS_INFO("\tDist. coef [0][3] %f", distortionCoeffs.ptr<double>(0)[3]);
  ROS_INFO("\tDist. coef [0][4] %f", distortionCoeffs.ptr<double>(0)[4]);

  // Create subscriber and publisher topics
  sub_topic = "/" + camera_name + "/image_raw";
  pub_topic = "/" + camera_name + "/image_undistorted";

  raw_image_sub = nh.subscribe<sensor_msgs::Image>(sub_topic, 1, &UndistortCamera::ImageCB, this);
  undistorted_pub = nh.advertise<sensor_msgs::Image>(pub_topic, 1);

  // Speed up undistort process by calculting the map1 and map2 variables once:
  // This function is called within the cv::undistort function, yet it takes about 90% of the computation time (about 90 ms).
  // Just do it once and speed up the process by a lot!
  initUndistortRectifyMap(cameraMatrix, distortionCoeffs, Mat(), cameraMatrix, img_size, CV_16SC2, map1, map2);
}

// Load parameter from namespace
template <typename T>
void UndistortCamera::LoadParam(string param, T &var)
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
    ROS_ERROR("Undistort Camera Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
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

  /*// Rotate image 180 degrees for downward camera based on how its plositioned
  if(strcmp(camera_name.c_str(), "downward") == 0) {
    rotate(imageUndistorted, imageUndistorted, ROTATE_180);
  }*/

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
