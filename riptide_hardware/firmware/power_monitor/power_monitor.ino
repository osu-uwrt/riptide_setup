#include <ros.h>
#include <riptide_msgs/BatStamped.h>

ros::NodeHandle nh;
riptide_msgs::BatStamped battery;

ros::Publisher state_pub("state/batteries", &battery);

void setup()
{
  nh.initNode();
  nh.advertise(state_pub);
}

void loop()
{
  nh.spinOnce();

  battery.port.current = analogRead(A3)*0.03255;
  battery.port.voltage = analogRead(A2)*0.03015;
  battery.stbd.current = analogRead(A0)*0.03255;
  battery.stbd.voltage = analogRead(A1)*0.03016;

  state_pub.publish(&battery);

  delay(10);
}
