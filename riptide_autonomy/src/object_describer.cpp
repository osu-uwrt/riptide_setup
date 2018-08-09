#include "riptide_autonomy/object_describer.h"

ObjectDescriber::ObjectDescriber(BeAutonomous *master)
{
  this->master = master;
}

void ObjectDescriber::GetRouletteHeading(void (Roulette::*callback)(double), Roulette *object)
{
  bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &ObjectDescriber::BBoxCB, this);
  image_sub = master->nh.subscribe<sensor_msgs::Image>("/downward/image_undistorted", 1, &ObjectDescriber::ImageCB, this);
  rouletteCallbackFunction = callback;
  rouletteCallbackObject = object;
  rouletteImagesLeft = IMAGES_TO_SAMPLE;
  averageRouletteAngle = 0;
}

void ObjectDescriber::GetPathHeading(void (PathMarker::*callback)(double), PathMarker *object)
{
  ROS_INFO("Get path");
  if(pathImagesLeft == 0)
  {
    bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &ObjectDescriber::BBoxCB, this);
    image_sub = master->nh.subscribe<sensor_msgs::Image>("/downward/image_undistorted", 1, &ObjectDescriber::ImageCB, this);
    pathCallbackFunction = callback;
    pathCallbackObject = object;
    pathImagesLeft = IMAGES_TO_SAMPLE;
    averagePathAngle = 0;
  }
}

void ObjectDescriber::ImageCB(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // Use the BGR8 image_encoding for proper color encoding
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  lastImage = cv_ptr->image.clone();
}

void ObjectDescriber::BBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bboxes)
{
  if (lastImage.size().width != 0 && lastImage.size().height != 0)
  {
    darknet_ros_msgs::BoundingBox bbox = bboxes->bounding_boxes[0];
    int64 width = bbox.xmax - bbox.xmin;
    int64 height = bbox.ymax - bbox.ymin;
    cv::Rect myROI(bbox.xmin, bbox.ymin, width, height);
    Mat cropped = lastImage(myROI);
    if (rouletteImagesLeft > 0)
    {
      Mat a;
      cropped.convertTo(a, CV_16S);
      Mat bgr[3];    //destination array
      split(a, bgr); //split source

      Mat values = bgr[0] + bgr[1];

      double min, max;
      minMaxLoc(values, &min, &max);

      Mat g = ((values - min) / (max - min) * 255);
      g.convertTo(values, CV_8U);

      double maxRadius = std::min(width, height) / 2 - 1;

      double scores[180];

      for (int angle = 0; angle < 180; angle++)
      {
        scores[angle] = 0;
        double radians = angle / 180.0 * 3.14159265;
        double dx = cos(radians);
        double dy = -sin(radians);

        for (int r = 0; r < maxRadius; r += 2)
        {
          int y = dy * r;
          int x = dx * r;
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

      if (--rouletteImagesLeft == 0)
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
    if (pathImagesLeft > 0)
    {
      Mat a;
      cropped.convertTo(a, CV_16S);
      Mat bgr[3];    //destination array
      split(a, bgr); //split source

      Mat values = bgr[2] - (bgr[0] + bgr[1]);

      double min, max;
      minMaxLoc(values, &min, &max);

      Mat g = ((values - min) / (max - min) * 255);
      g.convertTo(values, CV_8U);

      Scalar mean, dev;
      meanStdDev(values, mean, dev);

      threshold(values, values, mean[0] + dev[0] / 2, 255, 0);

      vector<vector<cv::Point>> contours;
      findContours(values, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      vector<cv::Point> largestContour;
      int largestArea = 0;
      for (int c = 0; c < contours.size(); c++)
      {
        if (contourArea(contours[c]) > largestArea)
        {
          largestContour = contours[c];
          largestArea = contourArea(contours[c]);
        }
      }

      vector<int> hull;
      convexHull(largestContour, hull);

      vector<Vec4i> defects;

      if (hull.size() > 3) // You need more than 3 indices
      {
        convexityDefects(largestContour, hull, defects);
      }

      Vec4i largestDefect;
      float largestDepth = 0;

      for (int i = 0; i < defects.size(); i++)
      {
        Vec4i v = defects[i];
        float depth = v[3];
        if (depth > largestDepth) //  filter defects by depth, e.g more than 10
        {
          largestDefect = v;
          largestDepth = depth;
        }
      }

      int startidx = largestDefect[0];
      cv::Point ptStart(largestContour[startidx]);
      int endidx = largestDefect[1];
      cv::Point ptEnd(largestContour[endidx]);

      double angle = atan2(ptStart.x - ptEnd.x, ptStart.y - ptEnd.y) / 3.1415926535897932384626433832795028841 * 180;

      double currentAverage = averagePathAngle / (IMAGES_TO_SAMPLE - pathImagesLeft);

      // Check for edge case where it goes from 1 degree to 179 degrees (-1 degrees)
      if (angle - currentAverage > 90)
        angle -= 180;
      else if (currentAverage - angle > 90)
        angle += 180;

      averagePathAngle += angle;

      if (--pathImagesLeft == 0)
      {
        bbox_sub.shutdown();
        image_sub.shutdown();
        averagePathAngle /= IMAGES_TO_SAMPLE;
        if (averagePathAngle < -180)
          averagePathAngle += 360;
        if (averagePathAngle > 180)
          averagePathAngle -= 360;
        (pathCallbackObject->*pathCallbackFunction)(averagePathAngle);
      }
    }
  }
}
