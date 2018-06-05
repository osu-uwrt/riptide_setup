#include "riptide_vision/rosbag_to_mp4.h"
using namespace cv;

ros::Nodehandle nh;
std::string topic, fileName, fileNameFull, camera;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosbag_to_mp4");
  RosbagToMP4 rtm;
  nh.param<std::string>("/rosbag_to_mp4/camera", camera, (std::string)"forward");
  nh.param<std::string>("/rosbag_to_mp4/file_name", fileName, (std::string)"pool_test.mp4");
  topic = "/" + camera + "/image_raw";
  fileNameFull = "~/" + fileName;
  VideoWriter videoWriter(fileNameFull, VideoWriter::fourcc('M', 'P', '4', '2'), );
  ros::spin();
}

RosbagToMP4::RosbagToMP4(cv::VideoWriter vWriter) {
  image_sub = nh.subscribe<sensor_msgs::Image>("/forward/image_color", 1, &RosbagToMP4::ImageCB, this);
}

void RosbagToMP4::WriteVideo(const sensor_msgs::Image::ConstPtr &msg) {

}
