#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;
std_msgs::Int32MultiArray state_msg;

int PINS = 10;
int PIN[] = { 2, 3, 4, 5, 8, 9, 10, 11, 12, 13};
int state[] = { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0};

void callback(const std_msgs::Int32 &cmd)
{
  int command = cmd.data;
  state[command] = !state[command];
  digitalWrite(PIN[command], state[command]);
}

ros::Publisher state_pub("state", &state_msg);
ros::Subscriber<std_msgs::Int32> command_sub("command", &callback);

void setup()
{
  nh.initNode();
  nh.advertise(state_pub);

  state_msg.data_length = PINS;
  
  state_msg.layout.dim_length = 1;
  state_msg.layout.data_offset = 0;

  state_msg.layout.dim[0].label = "state";
  state_msg.layout.dim[0].size = PINS;
  state_msg.layout.dim[0].stride = PINS;

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8, OUTPUT);

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(8, LOW);

  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}

void loop()
{
  nh.spinOnce();

  for(int i = 0; i < PINS; i++)
  {
    state_msg.data[i] = state[i];
  }

  state_pub.publish(&state_msg);
  delay(33);
}
