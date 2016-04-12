#include "ros/ros.h"
#include "riptide_msgs/PwmStamped.h"
#include <boost/asio.hpp>
#include <boost/thread.hpp>

class SerialClass{
  public:
    SerialClass():
    port(io),
            quitFlag(false){};

        ~SerialClass()
        {
            //Stop the I/O services
            io.stop();
            //Wait for the thread to finish
            runner.join();
        }

        bool connect(const std::string& port_name, int baud = 9600)
        {
            using namespace boost::asio;
            port.open(port_name);
            //Setup port
            port.set_option(serial_port::baud_rate(baud));
            port.set_option(serial_port::flow_control(
                    serial_port::flow_control::none));

            if (port.is_open())
            {
                //Start io-service in a background thread.
                //boost::bind binds the ioservice instance
                //with the method call
                runner = boost::thread(
                        boost::bind(
                                &boost::asio::io_service::run,
                                &io));

                startReceive();
            }

            return port.is_open();
        }

        void startReceive()
        {
            using namespace boost::asio;
            //Issue a async receive and give it a callback
            //onData that should be called when "\r\n"
            //is matched.
            async_read_until(port, buffer,
                    "\r\n",
                    boost::bind(&SerialClass::onData,
                            this, _1,_2));
        }

        void send(unsigned char *text, int SIZE)
        {

            boost::asio::write(port, boost::asio::buffer(text,SIZE));
        }

        void onData(const boost::system::error_code& e,
                std::size_t size)
        {
            if (!e)
            {
                std::istream is(&buffer);
                std::string data(size,'\0');
                is.read(&data[0],size);

                std::cout<<"Received data:"<<data;

                //If we receive quit()\r\n indicate
                //end of operations
                quitFlag = (data.compare("quit()\r\n") == 0);
            };

            startReceive();
        };

        bool quit(){return quitFlag;}

    private:
        boost::asio::io_service io;
        boost::asio::serial_port port;

        boost::thread runner;
        boost::asio::streambuf buffer;

        bool quitFlag;
    };


    int main(int argc, char* argv[])
    {
        SerialClass serial;

        if (serial.connect("/dev/ttyUSB0", 115200))
        {
            std::cout<<"Port is open."<<std::endl;
        }
        else
        {
            std::cout<<"Port open failed."<<std::endl;
        }

        while (!serial.quit())
        {
        int SIZE = 21;
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

            serial.send(&packet, SIZE);
            usleep(1000*500);
        }

        return 0;
    }




