#include <avr/io.h>
#include <avr/interrupt.h>
#include "TWI_slave.c"

void init_pwm() {
  //set fast pwm mode
  DDRA = 0b01100000;
  PUEA = 0b00000000;

  TCCR1A = 0b00000010;
  TCCR1B = 0b00011001;
  ICR1H = 0b1001110;

  ICR1L = 0b00100000;
  TIMSK = 0b11100000;

  OCR1AH = 0b101;
  OCR1AL = 0b11011100;
  OCR1BH = 0b101;
  OCR1BL = 0b11011100;
}

void main()
{
  unsigned char I2C_address = 0b1000;
  init_pwm();
  TWI_Slave_Initialise(I2C_address);




  sei();

  TWI_Start_Transceiver();

  for(;;)
  {


  }
}






ISR(TIM1_COMPA_vect)
{
  //turn PWM pin 1 off
  PORTA &= 0b10111111;

  OCR1AH = TWI_buf[0];
  OCR1AL = TWI_buf[1];

}

ISR(TIM1_COMPB_vect)
{
  //turn PWM pin 0 off
  PORTA &= 0b11011111;
  OCR1BH = TWI_buf[2];
  OCR1BL = TWI_buf[3];
}

ISR(TIM1_OVF_vect)
{
  //turn both pins on
  PORTA |= 0b01100000;
}
