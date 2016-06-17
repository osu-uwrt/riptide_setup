#include <ros.h>

ros::NodeHandle nh;
std_msgs::Float64 state;

void callback(const std_msgs::Float64 cmd);
{

}

ros::Publisher state_pub("state", &state);
ros::Subscriber<std_msgs::Float64> cmd_sub("command", &callback);

void setup()
{
  nh.initNode();
  nh.advertise(state_pub);

}

void loop()
{
  nh.spinOnce();

  state[0] = port_servo_pos;
  state[1] = stbd_servo_pos;

  state_pub.publish(&state);

  delay(33);
}
