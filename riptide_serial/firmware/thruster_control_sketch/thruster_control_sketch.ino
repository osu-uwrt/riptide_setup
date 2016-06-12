#include <Wire.h>

#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

std_msgs::Int8 state;

ros::Publisher state_pub("state", &state);

void callback(const std_msgs::Int8 &cmd)
{
  state.data = cmd.data;
  state_pub.publish(&state);

  Wire.beginTransmission(3);
  Wire.write(0x10);
  Wire.write(0x10);
  Wire.write(0x10);
  Wire.endTransmission();

  delay(5);

  Wire.beginTransmission(3);
  Wire.write(0x20);
  Wire.write(0x20);
  Wire.write(0x20);
  Wire.endTransmission();

  delay(5);

  Wire.requestFrom(0x10, 4);

  while(Wire.available())
  {
    char c = Wire.read();
    state.data = c;
    state_pub.publish(&state);
  }
}

ros::Subscriber<std_msgs::Int8> cmd_sub("command", &callback);

void setup()
{
  Wire.begin();

  nh.initNode();
  nh.advertise(state_pub);
  nh.subscribe(cmd_sub);
}

void loop()
{ 
  nh.spinOnce();

  delay(33);
}
