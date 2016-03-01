#include <Servo.h>
#include <ax12.h>

const int PORT_SERVO = 18;
const int STBD_SERVO = 15;
const int AFT_THRUSTER = 13;
const int PORT_THRUSTER = 12;
const int STBD_THRUSTER = 14;

const int MIN_THRUST = 1000;
const int ZERO_THRUST = 1500;
const int MAX_THRUST = 2000;

const int PACKET_SIZE = 10;
byte packet[PACKET_SIZE];

Servo aft_thruster;
Servo port_thruster;
Servo stbd_thruster;

const float ANGLE_CONVERSION = 4096.0/360.0;

int heartbeat = 0;

void setup()
{
  Serial.begin(9600);

  ax12Init(1000000);

  SetPosition(PORT_SERVO, 90 * ANGLE_CONVERSION);
  SetPosition(STBD_SERVO, 90 * ANGLE_CONVERSION);

  aft_thruster.attach(AFT_THRUSTER);
  port_thruster.attach(PORT_THRUSTER);
  stbd_thruster.attach(STBD_THRUSTER);

  aft_thruster.writeMicroseconds(ZERO_THRUST);
  port_thruster.writeMicroseconds(ZERO_THRUST);
  stbd_thruster.writeMicroseconds(ZERO_THRUST);
  
  pinMode(0, OUTPUT);
  digitalWrite(0, heartbeat);
}

void loop()
{
  if(Serial.available() > PACKET_SIZE && Serial.read() == '-')
  {
    heartbeat = !heartbeat;
    digitalWrite(0, heartbeat);

    for(int i = 0; i < PACKET_SIZE; i++)
    {
      packet[i] = Serial.read();
    }
    int port_angle = (packet[0] << 8) | packet[1];
    int stbd_angle = (packet[2] << 8) | packet[3];
    int aft_power = (packet[4] << 8) | packet[5];
    int port_power = (packet[6] << 8) | packet[7];
    int stbd_power = (packet[8] << 8) | packet[9];

    SetPosition(PORT_SERVO, port_angle * ANGLE_CONVERSION);
    SetPosition(STBD_SERVO, stbd_angle * ANGLE_CONVERSION);

    aft_thruster.writeMicroseconds(aft_power);
    port_thruster.writeMicroseconds(port_power);
    stbd_thruster.writeMicroseconds(stbd_power);
  }
}

