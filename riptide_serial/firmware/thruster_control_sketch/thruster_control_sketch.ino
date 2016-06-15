#include <Wire.h>

#include <ros.h>
#include <riptide_msgs/PwmStamped.h>

ros::NodeHandle nh;

riptide_msgs::Pwm checksum;

ros::Publisher state_pub("checksum", &checksum);

void callback(const riptide_msgs::PwmStamped &cmd)
{
  checksum.f1 = -512;
  checksum.f2 = -512;
  checksum.f3 = -512;
  checksum.f4 = -512;
  checksum.u1 = -512;
  checksum.u2 = -512;
  checksum.u3 = -512;
  checksum.u4 = -512;
  checksum.s1 = -512;
  checksum.s2 = -512;

  Wire.beginTransmission(1);
  Wire.write(cmd.pwm.f2>>8); //port
  Wire.write(cmd.pwm.f2 %256);
  Wire.write(cmd.pwm.f1>>8);
  Wire.write(cmd.pwm.f1%256);
  Wire.endTransmission();

  Wire.requestFrom(1, 1);
  checksum.f2  = Wire.read();
  checksum.f1 = checksum.f2;

  Wire.beginTransmission(2);
  Wire.write(cmd.pwm.f3>>8);
  Wire.write(cmd.pwm.f3%256);
  Wire.write(cmd.pwm.f4>>8);
  Wire.write(cmd.pwm.f4%256);
  Wire.endTransmission();

  Wire.requestFrom(2, 1);
  checksum.f3  = Wire.read();
  checksum.f4 = checksum.f3;

  Wire.beginTransmission(4);
  Wire.write(cmd.pwm.s1>>8);
  Wire.write(cmd.pwm.s1%256);
  Wire.write(cmd.pwm.s2>>8);
  Wire.write(cmd.pwm.s2%256);
  Wire.endTransmission();

  Wire.requestFrom(4, 1);
  checksum.s1  = Wire.read();
  checksum.s2 = checksum.s1;

  Wire.beginTransmission(8);
  Wire.write(cmd.pwm.u4>>8);
  Wire.write(cmd.pwm.u4%256);
  Wire.write(cmd.pwm.u3>>8);
  Wire.write(cmd.pwm.u3%256);
  Wire.endTransmission();

  Wire.requestFrom(8, 1);
  checksum.u4  = Wire.read();
  checksum.u3 = checksum.u4;

  Wire.beginTransmission(16);
  Wire.write(cmd.pwm.u1>>8);
  Wire.write(cmd.pwm.u1%256);
  Wire.write(cmd.pwm.u2>>8);
  Wire.write(cmd.pwm.u2%256);
  Wire.endTransmission();

  Wire.requestFrom(16, 100);
  checksum.u1  = Wire.read();
  checksum.u2 = checksum.u1;

  state_pub.publish(&checksum);
  if(cmd.pwm.f1 != 1500) { digitalWrite(2, HIGH); }
  else {digitalWrite(2, LOW); }
  if(cmd.pwm.f2 != 1500) { digitalWrite(3, HIGH); }
  else {digitalWrite(3, LOW); }
  if(cmd.pwm.f3 != 1500) { digitalWrite(4, HIGH); }
  else {digitalWrite(4, LOW); }
  if(cmd.pwm.f4 != 1500) { digitalWrite(5, HIGH); }
  else {digitalWrite(5, LOW); }
  if(cmd.pwm.u1 != 1500) { digitalWrite(6, HIGH); }
  else {digitalWrite(6, LOW); }
  if(cmd.pwm.u2 != 1500) { digitalWrite(7, HIGH); }
  else {digitalWrite(7, LOW); }
  if(cmd.pwm.u3 != 1500) { digitalWrite(8, HIGH); }
  else {digitalWrite(8, LOW); }
  if(cmd.pwm.u4 != 1500) { digitalWrite(9, HIGH); }
  else {digitalWrite(9, LOW); }
  if(cmd.pwm.s1 != 1500) { digitalWrite(10, HIGH); }
  else {digitalWrite(10, LOW); }
  if(cmd.pwm.s2 != 1500) { digitalWrite(11, HIGH); }
  else {digitalWrite(11, LOW); }
  
}

ros::Subscriber<riptide_msgs::PwmStamped> cmd_sub("thrust_cal/pwm", &callback);

void setup()
{
  Wire.begin();

  nh.initNode();
  nh.advertise(state_pub);
  nh.subscribe(cmd_sub);
  
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
          
  
}

void loop()
{
  nh.spinOnce();

  delay(33);
}
