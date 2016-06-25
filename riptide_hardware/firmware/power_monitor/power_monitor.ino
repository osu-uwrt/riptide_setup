#include <ros.h>
#include <riiptide_msgs/BatStamped.h>

ros::NodeHandle nh;
riptide_msgs::BatStamped batteries;

ros::Publisher state_pub("state/batteries", &state);

void setup()
{
  nh.initNode();
  nh.advertise(state_pub);
}

void loop()
{
  nh.spinOnce();

  battery.port.current = analogRead(A3);
  battery.port.voltage = analogRead(A2);
  battery.stbd.current = analogRead(A0);
  battery.stbd.voltage = analogRead(A1);

  state_pub.publish(&battery);

  delay(10);
}
