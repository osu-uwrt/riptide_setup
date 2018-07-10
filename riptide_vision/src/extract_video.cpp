#include "riptide_vision/extract_video.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "extract_video");
  ExtractVideo ev;
  ev.Loop();
}

ExtractVideo::ExtractVideo() : nh("extract_video") {
  ExtractVideo::LoadParam<string>("topic", topic);
  ExtractVideo::LoadParam<string>("username", username);
  ExtractVideo::LoadParam<string>("file_name", file_name);
  ExtractVideo::LoadParam<string>("ext", ext);
  ExtractVideo::LoadParam<int>("frame_rate", frame_rate);
  ExtractVideo::LoadParam<int>("width", width); // Max 1288
  ExtractVideo::LoadParam<int>("height", height); // Max 964

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

// Load parameter from namespace
template <typename T>
void ExtractVideo::LoadParam(string param, T &var)
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
    ROS_ERROR("Extract Video Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
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
