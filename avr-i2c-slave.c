#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "avr-i2c-slave.h"
 
 
extern volatile uint8_t i2c_registerMap[];
extern volatile uint8_t i2c_registerAttributes[];
extern uint8_t i2c_registerMapSize;
static volatile uint8_t i2c_registerIdx=0;
static uint8_t i2c_rxIdx=0;
static uint8_t i2c_txIdx=0;
 
volatile I2CState i2c_state = I2C_NO_STATE;  // State byte. Default set to I2C_NO_STATE.

// This is true when the TWI is in the middle of a transfer
// and set to false when all bytes have been transmitted/received
// Also used to determine how deep we can sleep.
volatile uint8_t i2c_busy = 0;

void i2c_slave_init(uint8_t i2c_address, uint8_t i2c_all_call)
{
	i2c_state = I2C_NO_STATE;
	TWBR = I2C_TWBR;
	TWAR = ((i2c_address<<1) & 0xFE) | (i2c_all_call?1:0);                            // Set own TWI slave address. Accept TWI General Calls.
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
	i2c_busy = 0;
}    
    
/****************************************************************************
Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
until the I2C_ISR has completed with the previous operation. If there was an error, then the function 
will return the TWI State code. 
****************************************************************************/
I2CState i2c_get_state(void)
{
  while (i2c_busy);            // Wait until TWI has completed the transmission.
  return ( i2c_state );                         // Return error state. 
}

ISR(TWI_vect)
{
	uint8_t i;
	switch (TWSR)
	{
		case I2C_STX_ADR_ACK:              // Own SLA+R has been received; ACK has been returned
			i2c_txIdx   = i2c_registerIdx; // Set buffer pointer to first data location
		case I2C_STX_DATA_ACK:             // Data byte in TWDR has been transmitted; ACK has been received
			if (i2c_txIdx < i2c_registerMapSize)
				TWDR = i2c_registerMap[i2c_txIdx++];
			else
				TWDR = 0xFF;
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			i2c_busy = 1;
			break;

		case I2C_STX_DATA_NACK:          // Data byte in TWDR has been transmitted; NACK has been received. 
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			i2c_busy = 0;   // Transmit is finished, we are not busy anymore
			break;     

		case I2C_SRX_GEN_ACK:            // General call address has been received; ACK has been returned
		case I2C_SRX_ADR_ACK:            // Own SLA+W has been received ACK has been returned
			i2c_rxIdx = 0;               // Set buffer pointer to first data location
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			i2c_busy = 1;
			break;

		case I2C_SRX_ADR_DATA_ACK:       // Previously addressed with own SLA+W; data has been received; ACK has been returned
		case I2C_SRX_GEN_DATA_ACK:       // Previously addressed with general call; data has been received; ACK has been returned
			i = TWDR;
			if (0 == i2c_rxIdx)
			{
				// First byte of a write, this will become our new register index
				i2c_rxIdx = i2c_registerIdx = i;

			} else if (i2c_rxIdx >= i2c_registerMapSize) {
				// NACK the SOB
				if (255 != i2c_rxIdx)
					i2c_rxIdx++;

			} else {
				// Subsequent byte of a write.  If register marked writable, write it
				if (!(i2c_registerAttributes[i2c_rxIdx] & I2CREG_ATTR_READONLY))
					i2c_registerMap[i2c_rxIdx]	= i;

				if (255 != i2c_rxIdx)
					i2c_rxIdx++;
			}
				
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			i2c_busy = 1;
			break;

		case I2C_SRX_STOP_RESTART:       // A STOP condition or repeated START condition has been received while still addressed as Slave    
                                                        // Enter not addressed mode and listen to address match
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);  // Enable TWI-interface and release TWI pins
			i2c_busy = 0;  // We are waiting for a new address match, so we are not busy
			break;           

		case I2C_SRX_ADR_DATA_NACK:      // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
		case I2C_SRX_GEN_DATA_NACK:      // Previously addressed with general call; data has been received; NOT ACK has been returned
		case I2C_STX_DATA_ACK_LAST_BYTE: // Last data byte in TWDR has been transmitted (TWEA = \930\94); ACK has been received
//    case I2C_NO_STATE              // No relevant state information available; TWINT = \930\94
		case I2C_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
			i2c_state = TWSR;                 //Store TWI State as errormessage, operation also clears noErrors bit
			TWCR = _BV(TWSTO) | _BV(TWINT); //Recover from I2C_BUS_ERROR, this will release the SDA and SCL pins thus enabling other devices to use the bus
			break;

		default:     
			i2c_state = TWSR;                                 // Store TWI State as errormessage, operation also clears the Success bit.      
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
      
			i2c_busy = 0; // Unknown status, so we wait for a new address match that might be something we can handle
			break;
	}
}


