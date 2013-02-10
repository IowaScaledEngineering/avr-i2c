/*************************************************************************
Title:    MRBus AVR I2C Slave Library
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     avr-i2c-slave.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes

    Much of this was shamelessly borrowed from Atmel's appnote AVR311.
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

void i2c_slave_init(uint8_t i2c_address, uint8_t i2c_all_call);

#define I2CREG_ATTR_READONLY  0x01
#define I2C_FREQ 400000
#define I2C_TWBR ( ((F_CPU) / (2UL * (I2C_FREQ))) - 8UL)


/****************************************************************************
  TWI State codes
****************************************************************************/
typedef enum
{
	// General TWI Master staus codes                      
	I2C_START                  = 0x08,  // START has been transmitted  
	I2C_REP_START              = 0x10,  // Repeated START has been transmitted
	I2C_ARB_LOST               = 0x38,  // Arbitration lost

	// TWI Master Transmitter staus codes                      
	I2C_MTX_ADR_ACK            = 0x18,  // SLA+W has been tramsmitted and ACK received
	I2C_MTX_ADR_NACK           = 0x20,  // SLA+W has been tramsmitted and NACK received 
	I2C_MTX_DATA_ACK           = 0x28,  // Data byte has been tramsmitted and ACK received
	I2C_MTX_DATA_NACK          = 0x30,  // Data byte has been tramsmitted and NACK received 

	// TWI Master Receiver staus codes  
	I2C_MRX_ADR_ACK            = 0x40,  // SLA+R has been tramsmitted and ACK received
	I2C_MRX_ADR_NACK           = 0x48,  // SLA+R has been tramsmitted and NACK received
	I2C_MRX_DATA_ACK           = 0x50,  // Data byte has been received and ACK tramsmitted
	I2C_MRX_DATA_NACK          = 0x58,  // Data byte has been received and NACK tramsmitted

	// TWI Slave Transmitter staus codes
	I2C_STX_ADR_ACK            = 0xA8,  // Own SLA+R has been received; ACK has been returned
	I2C_STX_ADR_ACK_M_ARB_LOST = 0xB0,  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
	I2C_STX_DATA_ACK           = 0xB8,  // Data byte in TWDR has been transmitted; ACK has been received
	I2C_STX_DATA_NACK          = 0xC0,  // Data byte in TWDR has been transmitted; NOT ACK has been received
	I2C_STX_DATA_ACK_LAST_BYTE = 0xC8,  // Last data byte in TWDR has been transmitted; ACK has been received

	// TWI Slave Receiver staus codes
	I2C_SRX_ADR_ACK            = 0x60,  // Own SLA+W has been received ACK has been returned
	I2C_SRX_ADR_ACK_M_ARB_LOST = 0x68,  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
	I2C_SRX_GEN_ACK            = 0x70,  // General call address has been received; ACK has been returned
	I2C_SRX_GEN_ACK_M_ARB_LOST = 0x78,  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
	I2C_SRX_ADR_DATA_ACK       = 0x80,  // Previously addressed with own SLA+W; data has been received; ACK has been returned
	I2C_SRX_ADR_DATA_NACK      = 0x88,  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
	I2C_SRX_GEN_DATA_ACK       = 0x90,  // Previously addressed with general call; data has been received; ACK has been returned
	I2C_SRX_GEN_DATA_NACK      = 0x98,  // Previously addressed with general call; data has been received; NOT ACK has been returned
	I2C_SRX_STOP_RESTART       = 0xA0,  // A STOP condition or repeated START condition has been received while still addressed as Slave

	// TWI Miscellaneous status codes
	I2C_NO_STATE               = 0xF8,  // No relevant state information available;
	I2C_BUS_ERROR              = 0x00  // Bus error due to an illegal START or STOP condition

} I2CState;


