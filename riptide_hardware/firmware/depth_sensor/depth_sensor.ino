#include <ros.h>
#include <riptide_msgs/Depth.h>
#include "MS5837.h"

#include "Wire.h"

MS5837 sensor;

ros::NodeHandle nh;
riptide_msgs::Depth depth;

ros::Publisher state_pub("state/depth", &depth);

void setup()
{
  nh.initNode();
  nh.advertise(state_pub);

  Wire.begin();

 sensor.init();

 sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
}

void loop()
{
  nh.spinOnce();

  sensor.read();

  depth.depth = sensor.depth();
  depth.temp = sensor.temperature();
  depth.pressure = sensor.pressure();
  depth.altitude = sensor.altitude();

  state_pub.publish(&depth);

  delay(40);
}
