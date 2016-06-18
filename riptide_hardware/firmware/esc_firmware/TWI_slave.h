/*****************************************************************************
* Atmel Corporation
* File              : TWI_slave.h
* Revision          : 2475
* Date              : 2007-09-20 12:00:43 +0200 (to, 20 sep 2007)
* Updated by        : mlarsson
* Support mail      : avr@atmel.com
* Supported devices : All devices with a TWI module can be used.
* AppNote           : AVR311 - TWI Slave Implementation
* Description       : Header file for TWI_slave.c
*                     Include this file in the application.
****************************************************************************/
/* General disabling of MISRA rules:
 * * (MISRA C rule 1) compiler is configured to allow extensions
 * * (MISRA C rule 111) bit fields shall only be defined to be of type unsigned int or signed int
 * * (MISRA C rule 37) bitwise operations shall not be performed on signed integer types
 * As it does not work well with 8bit architecture and/or IAR

 * Other disabled MISRA rules
 * * (MISRA C rule 109) use of union - overlapping storage shall not be used
 * * (MISRA C rule 61) every non-empty case clause in a switch statement shall be terminated with a break statement
*/

/****************************************************************************
  TWI Status/Control register definitions
****************************************************************************/
#include <stdint.h>
#define TWI_BUFFER_SIZE 5      // Set this to the largest message size that will be sent including address byte.

/****************************************************************************
  Global definitions
****************************************************************************/
union TWI_statusReg_t                       // Status byte holding flags.
{
   volatile uint8_t all;
   struct
   {
      volatile uint8_t lastTransOK:1;
      volatile uint8_t RxDataInBuf:1;
      volatile uint8_t genAddressCall:1;                        // TRUE = General call, FALSE = TWI Address;
      volatile uint8_t TxRequest:1;
      volatile uint8_t unusedBits:4;
   };
};

extern union TWI_statusReg_t TWI_statusReg;

extern volatile uint8_t TWI_msgReceivedSize; //Total number of bytes received

//static uint8_t dont_sleep = 0;

/****************************************************************************
  Function definitions
****************************************************************************/
void TWI_Slave_Initialise(uint8_t);
uint8_t TWI_Transceiver_Busy(void);
uint8_t TWI_Get_State_Info(void);
void TWI_Start_Transceiver_With_Data(uint8_t*, uint8_t);
void TWI_Start_Transceiver(void);
uint8_t TWI_Get_Data_From_Transceiver(uint8_t*, uint8_t);

//#pragma vector=TWI_vect
ISR(TWI_vect);

/****************************************************************************
  Bit and byte definitions
****************************************************************************/
#define TWI_READ_BIT  0   // Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS  1   // Bit position for LSB of the slave address bits in the init byte.
#define TWI_GEN_BIT   0   // Bit position for LSB of the general call bit in the init byte.

#define TRUE          1
#define FALSE         0

/****************************************************************************
  TWI State codes
****************************************************************************/


// TWI Slave Transmitter status codes
#define TWI_STX_ADR_ACK            0b01110011  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_NACK           0b01100011
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0b10100011  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0b10110011  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = '0'); ACK has been received

// TWI Slave Receiver status codes
#define TWI_SRX_ADR_ACK            0b01100001  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_NACK           0b01110001  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0b10110001  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0b10100001  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0b01100000  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available; TWINT = '0'
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition
