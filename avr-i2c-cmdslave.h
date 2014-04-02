/*************************************************************************
Title:    AVR Command-Based I2C Slave Library
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     avr-i2c-cmdslave.h
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2013 Michael Petersen & Nathan Holmes

    This is a command-based I2C slave library based on the SMBus protocol.
    It is not fully SMBus compatible, but does implement many of the bus
    protocols defined in section 5.5 of the SMBus specification, including
    optional Packet Error Correction (PEC).  See http://smbus.org/ for
    more details and the complete specification.  There is also optional
    support for paged commands, similar to that specified by PMBus.  See 
    http://pmbus.org/ for details.  Any trademarks related to SMBus and 
    PMBus are property of their respective organizations and no claim of 
    compatibility or compliance to any standard is being made here.

    Much of this work was shamelessly borrowed from the AVR slave library
    written by Nathan Holmes, which in turn was borrowed from Atmel's 
    appnote AVR311.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#define I2C_FREQ 400000
#define I2C_TWBR ( ((F_CPU) / (2UL * (I2C_FREQ))) - 8UL)

typedef struct
{
	uint8_t cmdCode;
	uint8_t attributes;
	uint8_t readBytes;
	uint8_t writeBytes;
	uint8_t *ramAddr;
} i2cCommand;

typedef struct
{
	uint8_t code;
	uint8_t page;
} CmdBuffer;

volatile uint8_t cmdQueueHead;
volatile uint8_t cmdQueueTail;
volatile uint8_t cmdQueueFull;
CmdBuffer cmdQueue[I2C_CMD_BUFFER_SIZE];

uint8_t i2cCmdQueueDepth(void);
uint8_t i2cCmdQueuePush(CmdBuffer* data);
uint8_t i2cCmdQueuePop(CmdBuffer* data);


// Defines for i2cCommand attributes
#define I2C_PAGED              0x80
#define I2C_ASCII              0x02
#define I2C_BLOCK              0x01

// Defines for i2c_registerIndex
#define I2C_UNSUPPORTED 0xFF

// Defines for i2c_state
#define I2C_STATE_ERROR         0x01

// Command specific defines
#define I2C_STATUS_CML_CMD_FAULT   0x80
#define I2C_STATUS_CML_DATA_FAULT  0x40
#define I2C_STATUS_CML_PEC_FAULT   0x20
#define I2C_STATUS_CML_I2C_FAULT   0x02


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

void i2c_slave_init(uint8_t i2c_address, uint8_t i2c_all_call);

#endif // I2C_SLAVE_H

