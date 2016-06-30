#include <avr/io.h>
#include <avr/interrupt.h>
#include "TWI_slave.c"

int analog_ptr = 0;

void init_pwm() {
  //set fast pwm mode
  DDRA = 0b01100000;//set pins as output
  PUEA = 0b00000000;//disable internal pull up

  TCCR1A = 0b00000010;//control register
  TCCR1B = 0b00011010;//control register

  ICR1H = 0b01001110;//TOP limit for counter
  ICR1L = 0b00100000;

  TIMSK = 0b11100000;//enables interrupts


  OCR1AH = 0b101;
  OCR1AL = 0b11011100;
  OCR1BH = 0b101;
  OCR1BL = 0b11011100;
}

void setup_adc()
{
  //turn on adc
  ADCSRA = 0b10000111;// (1<<ADEN)| (1<<ADIE); 0b10001111


}
void analogRead(uint8_t pin)
{
  ADMUX = pin;
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1 << ADSC));
}


void main()
{
  //use the watchdog timer

  //calibrate the internal ocillilator
  OSCCAL0 = 67;
    //addr #1    66
    //addr #2    66
    //addr #4    93
    //addr #8    46
    //addr #16   65
    //spare esc board 67
  //change the system clock prescaler
  CCP = 0xD8;
  CLKPR = 0b00000000;



  unsigned char I2C_address = 0b10;
  init_pwm();
  setup_adc();
  TWI_Slave_Initialise(I2C_address);




  sei();

  TWI_Start_Transceiver();


  for(;;)
  {

    // analogRead(1);//temp sensor 1
     analogRead(4);//current sensor 1
     OCR1AL = ADCL;
     OCR1AH = ADCH;
     //analogRead(0);//temp sensor 2
     //analogRead(5);//current sensor 2
  }
}






ISR(TIM1_COMPA_vect)
{
  //turn PWM pin 1 off
  PORTA &= 0b11011111;



}

ISR(TIM1_COMPB_vect)
{
  //turn PWM pin 0 off
  PORTA &= 0b10111111;

}

ISR(TIM1_OVF_vect)
{
  //turn both pins on
  PORTA |= 0b01100000;


}
