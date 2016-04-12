#include "ros/ros.h"
#include "boost/asio.hpp"
#include "riptide_msgs/PwmStamped.h"

boost::asio::io_service i_o;
boost::asio::serial_port s_p(i_o);

void callback(const riptide_msgs::PwmStamped::ConstPtr& force)
{
  const int SIZE = 21;
  unsigned char packet[SIZE];

  packet[0]  = '#';

  packet[1]  = force->pwm.f1 >> 8;
  packet[2]  = force->pwm.f1;

  packet[3]  = force->pwm.f2 >> 8;
  packet[4]  = force->pwm.f2;

  packet[5]  = force->pwm.f3 >> 8;
  packet[6]  = force->pwm.f3;

  packet[7]  = force->pwm.f4 >> 8;
  packet[8]  = force->pwm.f4;

  packet[9]  = force->pwm.u1 >> 8;
  packet[10]  = force->pwm.u1;

  packet[11]  = force->pwm.u2 >> 8;
  packet[12]  = force->pwm.u2;

  packet[13]  = force->pwm.u3 >> 8;
  packet[14]  = force->pwm.u3;

  packet[15]  = force->pwm.u4 >> 8;
  packet[16]  = force->pwm.u4;

  packet[17]  = force->pwm.s1 >> 8;
  packet[18]  = force->pwm.s1;

  packet[19]  = force->pwm.s2 >> 8;
  packet[20]  = force->pwm.s2;

  s_p.write_some(boost::asio::buffer(packet, SIZE));
}

void loop()
{
    ros::Rate rate(30); //rate needs to be fast enough to read every byte that comes in
    while(ros::ok())
    {
     const int SIZE = 1;
     unsigned char byte[SIZE];

     s_p.read_some(boost::asio::buffer(byte,SIZE));
     if (*byte == '#') {
     s_p.read_some(boost::asio::buffer(byte,SIZE));
	if (*byte == '@') {
	s_p.read_some(boost::asio::buffer(byte,SIZE));
	const int SIZE = int(*byte);
	unsigned char actuator_data[SIZE];
	s_p.read_some(boost::asio::buffer(actuator_data,SIZE));
	}
	else if (*byte == '+') {
	s_p.read_some(boost::asio::buffer(byte,SIZE));
	const int SIZE = int(*byte);
	unsigned char thruster_data[SIZE];
	s_p.read_some(boost::asio::buffer(thruster_data,SIZE));
	}
	else if (*byte == '%') {
        s_p.read_some(boost::asio::buffer(byte,SIZE));
        const int SIZE = int(*byte);
        unsigned char power_data[SIZE];
        s_p.read_some(boost::asio::buffer(power_data,SIZE));
        }
      }
     ros::spinOnce();
     rate.sleep();
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_port");

  std::string port_name;
  int baud_rate;

  ros::NodeHandle nh;
  ros::Subscriber sub;

  nh.param<std::string>("serial_port/name", port_name, "/dev/ttyUSB0");
  nh.param<int>("serial_port/rate", baud_rate, 9600);

  s_p.open(port_name);
  ROS_INFO("Serial port name: %s", port_name.c_str());
  s_p.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  ROS_INFO("Serial port rate: %i", baud_rate);

  sub = nh.subscribe<riptide_msgs::PwmStamped>("thrust_cal/pwm", 1, &callback);

  loop(); 
}


