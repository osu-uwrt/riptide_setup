#ifndef ANGULAR_DAMPING_H
#define ANGULAR_DAMPING_H

#include <math.h>
#include <vector>
#include "ros/ros.h"
#include "tf/transform_listener.h"

class AngularDamping
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber thrust_sub;
    ros::Publisher ang_damp_pub;

    tf::TransformListener *listener;
    tf::StampedTransform tf_surge[2];
    tf::StampedTransform tf_sway[2];
    tf::StampedTransform tf_heave[4];

  public:
   AngularDamping(char **argv, tf::TransformListener *listener_adr);
   void get_transform(vector *v, tf::StampedTransform *tform);
   void loop();
}
