#include <Wire.h>
#include <ros.h>
#include <riptide_msgs/PwmStamped.h>
#include <riptide_msgs/EscStamped.h>

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
riptide_msgs::Esc state;
ros::Publisher state_pub("state/esc", &state);
ros::Subscriber<riptide_msgs::PwmStamped> cmd_sub("command/pwm", &callback);

void setup()
{
  Wire.begin();

  nh.initNode();
  nh.advertise(state_pub);
  nh.subscribe(cmd_sub);

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
}

// Production ready function
void callback(const riptide_msgs::PwmStamped &cmd)
{
  // Create local checksums
  state.surge_hi = cmd.pwm.surge_port_hi ^ cmd.pwm.surge_stbd_hi;
  state.surge_lo = cmd.pwm.surge_port_lo ^ cmd.pwm.surge_stbd_lo;
  state.sway = cmd.pwm.sway_fwd ^ cmd.pwm.sway_aft;
  state.heave_fwd = cmd.pwm.heave_port_fwd ^ cmd.pwm.heave_stbd_fwd;
  state.heave_aft = cmd.pwm.heave_port_aft ^ cmd.pwm.heave_port_aft;

  //
  // Write PWM data & request remote checksums
  //

  Wire.beginTransmission(ESC_BOARD[1]);
  Wire.write(valid(cmd.pwm.surge_port_hi));
  Wire.write(valid(cmd.pwm.surge_stbd_hi));
  Wire.endTransmission();

  Wire.requestFrom(ESC_BOARD[1], ONE_BYTE);
  state.surge_hi ^= Wire.read();

  Wire.beginTransmission(ESC_BOARD[2]);
  Wire.write(valid(cmd.pwm.surge_port_lo));
  Wire.write(valid(cmd.pwm.surge_stbd_lo));
  Wire.endTransmission();

  Wire.requestFrom(ESC_BOARD[2], ONE_BYTE);
  state.surge_lo ^= Wire.read();

  Wire.beginTransmission(ESC_BOARD[3]);
  Wire.write(valid(cmd.pwm.sway_fwd));
  Wire.write(valid(cmd.pwm.sway_aft));
  Wire.endTransmission();

  Wire.requestFrom(ESC_BOARD[3], ONE_BYTE);
  state.sway ^= Wire.read();

  Wire.beginTransmission(ESC_BOARD[4]);
  Wire.write(valid(cmd.pwm.heave_port_fwd));
  Wire.write(valid(cmd.pwm.heave_stbd_fwd));
  Wire.endTransmission();

  Wire.requestFrom(ESC_BOARD[4], ONE_BYTE);
  state.heave_fwd ^= Wire.read();

  Wire.beginTransmission(ESC_BOARD[5]);
  Wire.write(valid(cmd.pwm.heave_port_aft));
  Wire.write(valid(cmd.pwm.heave_stbd_aft));
  Wire.endTransmission();

  Wire.requestFrom(ESC_BOARD[5], ONE_BYTE);
  state.heave_aft ^= Wire.read();

  // Publish checksum results
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
