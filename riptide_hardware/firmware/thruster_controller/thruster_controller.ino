#include <Wire.h>
#include <ros.h>
#include <riptide_msgs/PwmStamped.h>
#include <std_msgs/Int8.h>

// Thrusters off!
int STOP = 1500;
// Checksum size
int ONE_BYTE = 1;
// Addresses
int ESC_BOARD[] = {1, 2, 4, 8, 16};

// Function prototypes
int16_t valid(int16_t pwm);
void callback(const riptide_msgs::PwmStamped &cmd);

// ROS is the best
ros::NodeHandle nh;
std_msgs::Int8 state;
ros::Publisher state_pub("state/esc", &state);
ros::Subscriber<riptide_msgs::PwmStamped> cmd_sub("command/pwm", &callback);

void setup()
{
  Wire.begin();

  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(state_pub);

  // Surge LEDs
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  // Heave LEDs
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  // Sway LEDs
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop()
{
  // Check msgs for callback
  nh.spinOnce();

  delay(5);
}

// Production ready function
void callback(const riptide_msgs::PwmStamped &cmd)
{
  //
  // Write PWM data & request remote checksums
  //

  Wire.beginTransmission(ESC_BOARD[0]);
  Wire.write(valid(cmd.pwm.surge_port_hi) >> 8);
  Wire.write(valid(cmd.pwm.surge_port_hi));
  Wire.write(valid(cmd.pwm.surge_stbd_hi) >> 8);
  Wire.write(valid(cmd.pwm.surge_stbd_hi));
  Wire.endTransmission();

  Wire.requestFrom(ESC_BOARD[0], ONE_BYTE);

  Wire.beginTransmission(ESC_BOARD[1]);
  Wire.write(valid(cmd.pwm.surge_port_lo) >> 8);
  Wire.write(valid(cmd.pwm.surge_port_lo));
  Wire.write(valid(cmd.pwm.surge_stbd_lo) >> 8);
  Wire.write(valid(cmd.pwm.surge_stbd_lo));
  Wire.endTransmission();

  Wire.requestFrom(ESC_BOARD[1], ONE_BYTE);

  Wire.beginTransmission(ESC_BOARD[2]);
  Wire.write(valid(cmd.pwm.sway_fwd) >> 8);
  Wire.write(valid(cmd.pwm.sway_fwd));
  Wire.write(valid(cmd.pwm.sway_aft) >> 8);
  Wire.write(valid(cmd.pwm.sway_aft));
  Wire.endTransmission();

  Wire.requestFrom(ESC_BOARD[2], ONE_BYTE);

  Wire.beginTransmission(ESC_BOARD[3]);
  Wire.write(valid(cmd.pwm.heave_port_fwd) >> 8);
  Wire.write(valid(cmd.pwm.heave_port_fwd));
  Wire.write(valid(cmd.pwm.heave_stbd_fwd) >> 8);
  Wire.write(valid(cmd.pwm.heave_stbd_fwd));
  Wire.endTransmission();

  Wire.beginTransmission(ESC_BOARD[4]);
  Wire.write(valid(cmd.pwm.heave_port_aft) >> 8);
  Wire.write(valid(cmd.pwm.heave_port_aft));
  Wire.write(valid(cmd.pwm.heave_stbd_aft) >> 8);
  Wire.write(valid(cmd.pwm.heave_stbd_aft));
  Wire.endTransmission();

  state_pub.publish(&state);

  // Red LEDs
  if(cmd.pwm.surge_port_hi != STOP) { digitalWrite(2, HIGH); }
  else {digitalWrite(2, LOW); }
  if(cmd.pwm.surge_stbd_hi != STOP) { digitalWrite(3, HIGH); }
  else {digitalWrite(3, LOW); }
  if(cmd.pwm.surge_port_lo != STOP) { digitalWrite(4, HIGH); }
  else {digitalWrite(4, LOW); }
  if(cmd.pwm.surge_stbd_lo != STOP) { digitalWrite(5, HIGH); }
  else {digitalWrite(5, LOW); }

  // Amber LEDs
  if(cmd.pwm.heave_port_fwd != STOP) { digitalWrite(6, HIGH); }
  else {digitalWrite(6, LOW); }
  if(cmd.pwm.heave_stbd_fwd != STOP) { digitalWrite(7, HIGH); }
  else {digitalWrite(7, LOW); }
  if(cmd.pwm.heave_port_aft != STOP) { digitalWrite(8, HIGH); }
  else {digitalWrite(8, LOW); }
  if(cmd.pwm.heave_stbd_aft != STOP) { digitalWrite(9, HIGH); }
  else {digitalWrite(9, LOW); }

  // Blue LEDs
  if(cmd.pwm.sway_fwd != STOP) { digitalWrite(10, HIGH); }
  else {digitalWrite(10, LOW); }
  if(cmd.pwm.sway_aft != STOP) { digitalWrite(11, HIGH); }
  else {digitalWrite(11, LOW); }
}

// Ensure 1100 <= pwm <= 1900
int16_t valid(int16_t pwm)
{
  pwm = pwm > 1900 ? 1900 : pwm;
  pwm = pwm < 1100 ? 1100 : pwm;
  return pwm;
}
