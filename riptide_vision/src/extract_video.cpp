#include "riptide_vision/extract_video.h"
using namespace cv;

int main(int argc, char** argv) {
  ros::init(argc, argv, "extract_video");
  ExtractVideo ev;
  ev.Loop();
}

ExtractVideo::ExtractVideo() : nh("extract_video") {
  nh.getParam("topic", topic);
  nh.getParam("username", username);
  nh.getParam("file_name", file_name);
  nh.getParam("ext", ext);
  nh.getParam("frame_rate", frame_rate);
  nh.getParam("width", width); // Max 1288
  nh.getParam("height", height); // Max 964

  Size frame_size(width, height);
  file_path = "/home/" + username + "/rosbags/" + file_name + ext;
  videoWriter = VideoWriter(file_path, CV_FOURCC('M', 'J', 'P', 'G'), frame_rate, frame_size, true);

  image_sub = nh.subscribe<sensor_msgs::Image>(topic, 1, &ExtractVideo::WriteVideo, this);

  ROS_INFO("Ready to extract video.");
  ROS_INFO("Subscribing to topic: %s", topic.c_str());
  ROS_INFO("Writing video to file: %s", file_path.c_str());
  frames = 0;
}

ExtractVideo::~ExtractVideo() {
  videoWriter.release();
  destroyAllWindows();
  double total_sec = (frames*1.0) / frame_rate; // Multiply by 1.0 to perform "double" math
  int min = total_sec / 60;
  double sec = total_sec - min*60.0;
  printf("\nVideo duration: %.3f sec (%i min and %.3f sec)\n\n", total_sec, min, sec);
}

void ExtractVideo::WriteVideo(const sensor_msgs::Image::ConstPtr &msg) {
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  if(!cv_ptr->image.empty()) {
    if(videoWriter.isOpened()) {
      if(frames == 0) { // Verify frame sizes and notify user of any wrong values
        int w = cv_ptr->image.size().width;
        int h = cv_ptr->image.size().height;
        if(w != width || h != height) {
          ROS_ERROR("Publishing frame size of %ix%i does not match specified frame size of %ix%i", w, h, width, height);
          ros::shutdown();
        }
      }
      videoWriter.write(cv_ptr->image);
      frames++;
      ROS_INFO("Wrote %i frames", frames);
    }
  }
  cv_ptr.reset();
}

void ExtractVideo::Loop()
{
  ros::Rate rate(frame_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
