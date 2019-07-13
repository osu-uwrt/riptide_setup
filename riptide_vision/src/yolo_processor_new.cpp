#include "riptide_vision/yolo_processor_new.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_processor_new");
  YoloProcessorNew ypn;
  try
  {
    ros::spin();
  }
  catch (exception &e)
  {
    ROS_ERROR("YOLO Processor New Error: %s", e.what());
    ROS_ERROR("YOLO Processor New: Shutting Down");
  }
}


