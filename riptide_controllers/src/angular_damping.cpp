#include "riptide_controllers/angular_damping.h"

struct vector
{
  double x;
  double y;
  double z;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "angular_damping");
  AngularDamping ad;
  loop();
}

AngularDamping::AngularDamping(char **argv, tf::TransformListener *listener_adr) {

}

void AngularDamping::loop() {
  ros::Rate rate(1000);
  while(!ros::isShuttingDown) {
    ros::spinOnce();
    rate.sleep();
  }
}
