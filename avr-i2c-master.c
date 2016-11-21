/*************************************************************************
Title:    MRBus AVR I2C Library
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     avr-i2c-master.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes

    Much of this was shamelessly borrowed from Atmel's appnote AVR315.
    My thanks to them for saving me a great amount of time.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "avr-i2c-master.h"

volatile uint8_t i2c_buffer[I2C_MAX_BUFFER_SIZE];    // Transceiver buffer
uint8_t i2c_bufferLen = 0;                   // Number of bytes to be transmitted.
volatile uint8_t i2c_bufferIdx = 0;
volatile uint8_t i2c_state = I2C_NO_STATE;      // State byte. Default set to I2C_NO_STATE.
volatile uint8_t i2c_status = 0;

ISR(I2C_vect)
{
	switch (TWSR & 0xFC)
	{
		case I2C_START:             // START has been transmitted  
		case I2C_REP_START:         // Repeated START has been transmitted
			i2c_bufferIdx = 0;       // Set buffer pointer to the TWI Address location
		case I2C_MTX_ADR_ACK:       // SLA+W has been tramsmitted and ACK received
		case I2C_MTX_DATA_ACK:      // Data byte has been tramsmitted and ACK received
			if (i2c_bufferIdx < i2c_bufferLen)
			{
				TWDR = i2c_buffer[i2c_bufferIdx++];
				// TWI Interface enabled, enable TWI Interupt and clear the flag to send byte
				TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);    
			} else {                    // Send STOP after last byte
				i2c_status |= _BV(I2C_MSG_RECV_GOOD);
				// TWI Interface enabled, disable TWI Interrupt and clear the flag, send stop (if requested)
				if (i2c_status & _BV(I2C_MSG_SEND_STOP))
					TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWSTO);
				else
					TWCR = _BV(TWEN);
			}
			break;

		case I2C_MRX_DATA_ACK:      // Data byte has been received and ACK tramsmitted
			i2c_buffer[i2c_bufferIdx++] = TWDR;
		case I2C_MRX_ADR_ACK:       // SLA+R has been tramsmitted and ACK received
			// Detect the last byte to NACK it.
			if (i2c_bufferIdx < (i2c_bufferLen-1) )
			{
				// TWI Interface enabled, enable TWI Interupt and clear the flag to read next byte, send ACK after reception
				TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);  
			} else {                    // Send NACK after next reception
				// TWI Interface enabled, enable TWI Interupt and clear the flag to read next byte, send NACK after reception
				TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
			}
			break; 


		case I2C_MRX_DATA_NACK:     // Data byte has been received and NACK tramsmitted
			i2c_buffer[i2c_bufferIdx] = TWDR;
			i2c_status |= _BV(I2C_MSG_RECV_GOOD);               // Set status bits to completed successfully. 
			// TWI Interface enabled, disable TWI Interrupt and clear the flag, initiate stop
			TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWSTO);
			break;      

		case I2C_ARB_LOST:          // Arbitration lost
			// TWI Interface enabled, Enable TWI Interupt and clear the flag, Initiate a (RE)START condition.
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWSTA);
			break;

		case I2C_MTX_ADR_NACK:      // SLA+W has been tramsmitted and NACK received
		case I2C_MRX_ADR_NACK:      // SLA+R has been tramsmitted and NACK received    
		case I2C_MTX_DATA_NACK:     // Data byte has been tramsmitted and NACK received
			// Store TWSR and automatically sets clears noErrors bit.
			i2c_state = TWSR & 0xFC;
			// Send stop to clear things out since slave NACK'd
			TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWSTO);
			break;      
		case I2C_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
		case I2C_NO_STATE:          // No relevant state information available; TWINT
		default:     
			// Store TWSR and automatically sets clears noErrors bit.
			i2c_state = TWSR & 0xFC;
			// Reset TWI Interface
			TWCR = _BV(TWEN);
			break;
	}
}

void i2c_master_init(void)
{
	i2c_status = 0;
	i2c_state = I2C_NO_STATE;
	TWBR = I2C_TWBR;                                  // Set bit rate register (Baudrate). Defined in header file.
	TWSR = I2C_TWSR;                                  // Prescaler
	TWDR = 0xFF;                                      // Default content = SDA released.
	TWCR = _BV(TWEN);
}    

uint8_t i2c_busy(void)
{
	return( TWCR & (_BV(TWIE)) );
}

uint8_t i2c_transaction_successful()
{
	return((i2c_status & (_BV(I2C_MSG_RECV_GOOD))) ? 1:0);
}

/****************************************************************************
Call this function to send a prepared message. The first byte must contain the slave address and the
read/write bit. Consecutive bytes contain the data to be sent, or empty locations for data to be read
from the slave. Also include how many bytes that should be sent/read including the address byte.
The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
then initialize the next operation and return.
****************************************************************************/
void i2c_transmit(uint8_t *msgBuffer, uint8_t msgLen, uint8_t sendStop)
{
	// Wait until I2C isn't busy
	while ( i2c_busy() );             

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Number of data bytes to transmit
		i2c_bufferLen = msgLen;
		i2c_buffer[0] = msgBuffer[0];  // Destination slave address with R/W bit


		if (!(i2c_buffer[0] & (_BV(I2C_READ_BIT))))  // If it's a write, copy the rest of the bytes
			memcpy((uint8_t*)i2c_buffer+1, msgBuffer+1, msgLen-1);

		i2c_state = I2C_NO_STATE;
		i2c_status = 0;
		if (sendStop)
			i2c_status |= _BV(I2C_MSG_SEND_STOP);
	}
	// Enable interrupts and issue a start condition
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWSTA);
}

uint8_t i2c_receive(uint8_t *msgBuffer, uint8_t msgLen)
{
	// Wait until I2C isn't busy
	while ( i2c_busy() );
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (i2c_status & (_BV(I2C_MSG_RECV_GOOD)))
			memcpy(msgBuffer, (uint8_t*)i2c_buffer, msgLen);
	}

	return((i2c_status & (_BV(I2C_MSG_RECV_GOOD))) ? 1:0);
}


