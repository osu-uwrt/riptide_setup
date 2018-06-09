#include "riptide_hardware/calibrate_camera.h"
//using namespace cv;

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibrate_camera");
  CalibrateCamera cc;
  cc.Loop();
}

CalibrateCamera::CalibrateCamera() : nh("calibrate_camera") {
  nh.getParam("numBoards", numBoards);
  nh.getParam("numCornersHor", numCornersHor);
  nh.getParam("numCornersVer", numCornersVer);
  nh.getParam("frame_rate", frame_rate);
  nh.getParam("camera", camera);

  sub_topic = "/" + camera + "/image_raw";
  pub_topic = "/" + camera + "/undistored";
  raw_image_sub = nh.subscribe<sensor_msgs::Image>(sub_topic, 1, &CalibrateCamera::ImageCB, this);

  numSquares = numCornersHor * numCornersVer;
  board_sz = Size(numCornersHor, numCornersVer);

  for(int j=0;j<numSquares;j++)
    obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

  // Intrinsic camera matrix
  // Aspect ratio is 4:3
  intrinsic = Mat(3, 3, CV_32FC1);
  intrinsic.ptr<float>(0)[0] = 4; // Focal length along x-axis
  intrinsic.ptr<float>(1)[1] = 3; // Focal length along y-axis
  calculated = false;
  pauses = 0;
}

void CalibrateCamera::ImageCB(const sensor_msgs::Image::ConstPtr &msg) {
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  // Detect at rate slower than actual fps so user can move camera around
  if(!calculated) {
    pauses++;
    if(pauses > frame_rate/2)
      pauses = 0;
  }

  // Collect Test Data
  if(!cv_ptr->image.empty() && (successes <= numBoards)) {
    cvtColor(cv_ptr->image, gray_image, CV_BGR2GRAY);

    bool found = findChessboardCorners(cv_ptr->image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if(found)
    {
      cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(gray_image, board_sz, corners, found);
    }

    imshow("win1", cv_ptr->image);
    imshow("win2", gray_image);
    waitKey(1);

    if(pauses == 0) {
      image_points.push_back(corners);
      object_points.push_back(obj);
      ROS_INFO("Number Successes: %i", successes);
      successes++;
    }
  }

  // Calculate coefficients from chessboard data
  if(successes > numBoards && !calculated) {
    ROS_INFO("Calibrating Camera...");
    calibrateCamera(object_points, image_points, cv_ptr->image.size(), intrinsic, distCoeffs, rvecs, tvecs);
    ROS_INFO("Camera calibrated!");
    ROS_INFO("Intrinsic and Distortion Coefficients displayed below");

    std::cout << "Intrinsic:" << endl << intrinsic << endl << endl;
    std::cout << "Distortion Coeff:" << endl << distCoeffs << endl << endl;
    calculated = true;
  }

  // Publish undistorted image
  if(calculated) {
    Mat imageUndistorted;
    undistort(cv_ptr->image, imageUndistorted, intrinsic, distCoeffs);

    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageUndistorted).toImageMsg();

    imshow("win1", cv_ptr->image);
    imshow("win3", imageUndistorted);
    waitKey(1);
  }

  cv_ptr.reset();
}

void CalibrateCamera::Loop()
{
  ros::Rate rate(frame_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
