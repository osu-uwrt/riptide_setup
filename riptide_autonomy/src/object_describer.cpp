#include "riptide_autonomy/object_describer.h"


ObjectDescriber::ObjectDescriber(BeAutonomous* master) {
  this->master = master;
}

void ObjectDescriber::GetRouletteHeading(void (Roulette::*callback)(double), Roulette *object) {
  bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &ObjectDescriber::BBoxCB, this);
  image_sub = master->nh.subscribe<sensor_msgs::Image>("/downward/image_undistorted", 1, &ObjectDescriber::ImageCB, this);
  rouletteCallbackFunction = callback;
  rouletteCallbackObject = object;
  rouletteImagesLeft = IMAGES_TO_SAMPLE;
  averageRouletteAngle = 0;
}

void ObjectDescriber::ImageCB(const sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e ){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  lastImage = cv_ptr->image.clone();
}

void ObjectDescriber::BBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bboxes) {
  if(rouletteImagesLeft > 0 && lastImage.size().width != 0 && lastImage.size().height != 0) {
    darknet_ros_msgs::BoundingBox bbox = bboxes->bounding_boxes[0];
    int64 width = bbox.xmax - bbox.xmin;
    int64 height = bbox.ymax - bbox.ymin;
    cv::Rect myROI(bbox.xmin, bbox.ymin, width, height);
    Mat cropped = lastImage(myROI);
    Mat bgr[3];   //destination array
    split(cropped, bgr); //split source  


    Mat values = bgr[0] + bgr[1];

    double maxRadius = min(width, height) / 2 - 1;

    double scores[180];

    for (int angle = 0; angle < 180; angle++)
    {
      scores[angle] = 0;
      double radians = angle / 180.0 * 3.14159265;
      double dx = cos(radians);
      double dy = -sin(radians);

      for (int r = 0; r < maxRadius; r += 2)
      {
        int y = dy*r;
        int x = dx*r;
        scores[angle] += values.at<uchar>(height / 2 + y, width / 2 + x);
        scores[angle] += values.at<uchar>(height / 2 - y, width / 2 - x);
      }
    }


    int maxIndex = 0;
    int maxVal = 0;
    for (int i = 0; i < 180; i++)
    {
      int score = 0;
      for (int a = 0; a < 10; a++)
      {
        score += scores[(i + a) % 180];
        score += scores[(i - a + 180) % 180];
      }
      if (maxVal < score)
      {
        maxIndex = i;
        maxVal = score;
      }
    }

    double currentAverage = averageRouletteAngle / (IMAGES_TO_SAMPLE - rouletteImagesLeft);

    // Check for edge case where it goes from 1 degree to 179 degrees (-1 degrees)
    if (maxIndex - currentAverage > 90)
      maxIndex -= 180;
    else if (currentAverage - maxIndex > 90)
      maxIndex += 180;

    averageRouletteAngle += maxIndex;

    if(--rouletteImagesLeft == 0)
    {
      bbox_sub.shutdown();
      image_sub.shutdown();
      averageRouletteAngle /= IMAGES_TO_SAMPLE;
      if (averageRouletteAngle < 0)
        averageRouletteAngle += 180;
      if (averageRouletteAngle > 180)
        averageRouletteAngle -= 180;
      (rouletteCallbackObject->*rouletteCallbackFunction)(averageRouletteAngle);
    }
  }

  
}
