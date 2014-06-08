/*************************************************************************
Title:    AVR Command-Based I2C Slave Library
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     avr-i2c-cmdslave.c
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

// FIXME: Although mostly PEC independent, the lib currently has no way to *require* PEC on writes

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include "avr-i2c-cmdslave.h"

// I2C configuration provided by the application
extern i2cCommand i2c_registerMap[];
extern volatile uint8_t i2c_registerIndex[];
// Required commands since the library uses them
#ifdef I2C_ENABLE_PAGE
extern volatile uint8_t I2C_PAGE[1];
#else
volatile uint8_t I2C_PAGE[1];
#endif // I2C_ENABLE_PAGE
#ifdef I2C_ENABLE_STATUS_WORD
extern volatile uint16_t I2C_STATUS_WORD[I2C_NUMPAGES];
#else
volatile uint8_t I2C_STATUS_WORD[I2C_NUMPAGES];
#endif // I2C_ENABLE_STATUS_WORD
#ifdef I2C_ENABLE_CML
extern volatile uint8_t I2C_STATUS_CML[1];
#else
volatile uint8_t I2C_STATUS_CML[1];
#endif // I2C_ENABLE_CML

// Some helper macros
#ifdef I2C_ENABLE_PAGE
#define IS_PAGED           (i2c_registerMap[i2c_registerMapIndex].attributes & I2C_PAGED)
#else
#define IS_PAGED           (0)
#endif
#define IS_BLOCKCMD        (i2c_registerMap[i2c_registerMapIndex].attributes & I2C_BLOCK)
#define IS_LBLOCK          (i2c_registerMap[i2c_registerMapIndex].attributes & I2C_LEN)

// Internal index to the i2c_registerMap array
static volatile uint8_t i2c_registerMapIndex;

static uint8_t i2c_state;
static uint8_t i2c_pec;
static uint8_t i2c_baseAddress;
static uint8_t i2c_status;
static uint8_t i2c_buffer[256];
static CmdBuffer i2c_command;

uint8_t i2cCmdQueueDepth(void)
{
	uint8_t result = 0;
	if(cmdQueueFull)
		return(I2C_CMD_BUFFER_SIZE);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		result = (uint8_t)(cmdQueueHead - cmdQueueTail) % I2C_CMD_BUFFER_SIZE;
	}
	return(result);
}

uint8_t i2cCmdQueuePush(CmdBuffer* data)
{
	// If full, bail with a false
	if (cmdQueueFull)
		return(0);

	cmdQueue[cmdQueueHead].code = data->code;
	cmdQueue[cmdQueueHead].page = data->page;

	if( ++cmdQueueHead >= I2C_CMD_BUFFER_SIZE )
		cmdQueueHead = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (cmdQueueHead == cmdQueueTail)
			cmdQueueFull = 1;;
	}
	return(1);
}

uint8_t i2cCmdQueuePop(CmdBuffer* data)
{
	if (0 == i2cCmdQueueDepth())
		return(0);

	data->code = cmdQueue[cmdQueueTail].code;
	data->page = cmdQueue[cmdQueueTail].page;
		
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if( ++cmdQueueTail >= I2C_CMD_BUFFER_SIZE )
			cmdQueueTail = 0;
		cmdQueueFull = 0;
	}

	return(1);
}

// Table of crc values stored in flash.
const uint8_t crcTable[256] PROGMEM = {	
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
	0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
	0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
	0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
	0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
	0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
	0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
	0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
	0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
	0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
	0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

void i2c_calculatePec(uint8_t data)
{
	i2c_pec ^= data;
	i2c_pec = pgm_read_byte(&(crcTable[i2c_pec]));
}

static uint16_t i2c_rxIdx=0;  // Receive byte count (not including address)
static uint16_t i2c_txIdx=0;

static uint8_t writeBytes;  // Local storage of bytes to be written

// This is true when the TWI is in the middle of a transfer
// and set to false when all bytes have been transmitted/received
// Also used to determine how deep we can sleep.
volatile uint8_t i2c_busy = 0;

void i2c_slave_init(uint8_t i2c_address, uint8_t i2c_all_call)
{
	TWBR = I2C_TWBR;
	TWAR = ((i2c_address<<1) & 0xFE) | (i2c_all_call?1:0);                            // Set own TWI slave address. Accept TWI General Calls.
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
	i2c_busy = 0;
	i2c_baseAddress = i2c_address;
	i2c_pec = 0;
	I2C_STATUS_CML[0] = 0;
	
	// Initialize command queue
	cmdQueueHead = cmdQueueTail = 0;
	cmdQueueFull = 0;
	memset(cmdQueue, 0, sizeof(cmdQueue));
}    


// FIXME: Add ARA support?

ISR(TWI_vect)
{
	uint8_t data;
	uint16_t txIndex;
	i2c_status = 0;

	switch (TWSR)
	{
		case I2C_STX_ADR_ACK:              // Own SLA+R has been received; ACK has been returned
			i2c_txIdx = 1;                 // Initialize transmit byte count (1 based to be consistent with data byte count when receiving; 0 was slave addr)
			i2c_calculatePec((i2c_baseAddress << 1) + 1);
			// Fall through to next case in order to preload data byte
		case I2C_STX_DATA_ACK:             // Data byte in TWDR has been transmitted; ACK has been received
			txIndex = (IS_BLOCKCMD?i2c_txIdx-1:i2c_txIdx);  // Mangle index to handle block reads.  Do it once here.
			if (I2C_UNSUPPORTED == i2c_registerIndex[i2c_command.code])
			{
				TWDR = 0xFF;  // Drive 0xFF so bus is released
				i2c_txIdx++;
			}
			else if(0 == i2c_registerMap[i2c_registerMapIndex].readBytes)
			{
				// Send byte command (write only)
				i2c_status |= STATUS_CML_I2C_FAULT;
				TWDR = 0xFF;  // Drive 0xFF so bus is released
				i2c_txIdx++;
			}
			else if( IS_BLOCKCMD && (1 == i2c_txIdx) )
			{
				// Block read.  Return # of bytes.
				if(IS_LBLOCK)
				{
					uint16_t pageOffset;  // 16 bits to handle word reads with page > 127
					pageOffset = (IS_PAGED ? (I2C_PAGE[0] * (i2c_registerMap[i2c_registerMapIndex].readBytes + (IS_LBLOCK?1:0))) : 0);
					data = *(i2c_registerMap[i2c_registerMapIndex].ramAddr + pageOffset);
				}
				else
				{
					data = i2c_registerMap[i2c_registerMapIndex].readBytes;
				}
				TWDR = data;
				i2c_calculatePec(data);
				i2c_txIdx++;
			}
			else if(txIndex <= i2c_registerMap[i2c_registerMapIndex].readBytes)
			{
				if( IS_PAGED && (0xFF == I2C_PAGE[0]) )
				{
					// Handles reads of data from paged registers with PAGE = 0xFF (illegal)
					i2c_status |= STATUS_CML_DATA_FAULT;
					TWDR = 0x00;
				}
				else
				{
					uint16_t pageOffset;  // 16 bits to handle word reads with page > 127
					pageOffset = (IS_PAGED ? (I2C_PAGE[0] * (i2c_registerMap[i2c_registerMapIndex].readBytes + (IS_LBLOCK?1:0))) : 0);
					if(i2c_registerMap[i2c_registerMapIndex].attributes & I2C_SKIP_BYTE)
						pageOffset *= 2;  // Read byte size registers from word size source
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
					data = *(i2c_registerMap[i2c_registerMapIndex].ramAddr + pageOffset + (IS_LBLOCK?1:0) + txIndex - 1);
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
					// Mangle byte order since SMBus sends lowest byte first - Read from end to the beginning
					data = *(i2c_registerMap[i2c_registerMapIndex].ramAddr + pageOffset + i2c_registerMap[i2c_registerMapIndex].readBytes + (IS_LBLOCK?1:0) - txIndex);
#endif
					TWDR = data;
					i2c_calculatePec(data);
				}
				i2c_txIdx++;
			}
			else if(txIndex == (i2c_registerMap[i2c_registerMapIndex].readBytes + 1))
			{
				// Send PEC
				TWDR = i2c_pec;
				i2c_txIdx++;
			}
			else
			{
				// Too many bytes read, set status
				i2c_status |= STATUS_CML_I2C_FAULT;
				TWDR = 0xFF;  // Drive 0xFF so bus is released
				i2c_txIdx++;
			}
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			i2c_busy = 1;
			break;

		case I2C_STX_DATA_NACK:          // Data byte in TWDR has been transmitted; NACK has been received. 
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			i2c_busy = 0;   // Transmit is finished, we are not busy anymore
			break;     

		case I2C_SRX_GEN_ACK:            // General call address has been received; ACK has been returned
		case I2C_SRX_ADR_ACK:            // Own SLA+W has been received ACK has been returned
			i2c_pec = 0;
			i2c_state &= ~I2C_STATE_ERROR;  // Clear error flag
			i2c_calculatePec(i2c_baseAddress << 1);
			i2c_rxIdx = 0;               // Initialize receive byte count
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			break;

		case I2C_SRX_ADR_DATA_ACK:       // Previously addressed with own SLA+W; data has been received; ACK has been returned
		case I2C_SRX_GEN_DATA_ACK:       // Previously addressed with general call; data has been received; ACK has been returned
			data = TWDR;
			if (0 == i2c_rxIdx)
			{
				// First byte of a write, this is the command code
				i2c_command.code = data;  // Save for command processing in application or for validating a read operation
				i2c_command.page = I2C_PAGE[0];  // Save for command processing in application
				if(I2C_UNSUPPORTED == i2c_registerIndex[i2c_command.code])
				{
					// Set error if unsupported
					i2c_status |= STATUS_CML_CMD_FAULT;
				}
				else
				{
					i2c_registerMapIndex = i2c_registerIndex[i2c_command.code]; // Save command pointer for future processing
					writeBytes = i2c_registerMap[i2c_registerMapIndex].writeBytes;  // Save write bytes; block command will override later
					i2c_calculatePec(data);
				}
			}
			else if (I2C_UNSUPPORTED == i2c_registerIndex[i2c_command.code])
			{
				// Subsequent writes to unsupported command
				// Do nothing
			}
			else if (i2c_rxIdx > writeBytes)
			{
				// Write bigger than command size
				if(i2c_rxIdx == (writeBytes + 1))
				{
					// First extra byte... Maybe PEC?
					if( (i2c_pec != data) && ((0 != i2c_registerMap[i2c_registerMapIndex].writeBytes) || (0 == i2c_registerMap[i2c_registerMapIndex].readBytes)) )
					{
						// PEC doesn't match and it's a writeable command or a send byte command - throw an error
						if( !(i2c_state & I2C_STATE_ERROR) )
						{
							i2c_status |= STATUS_CML_PEC_FAULT;
						}
					}
					else if( (0 == i2c_registerMap[i2c_registerMapIndex].writeBytes) && (i2c_registerMap[i2c_registerMapIndex].readBytes > 0) )
					{
						// Read-only command - throw a different error
						i2c_status |= STATUS_CML_DATA_FAULT;
					}
				}
				else
				{
					// Beyond PEC, throw an error (unless already flagged as PEC mismatch)
					if( !(i2c_state & I2C_STATE_ERROR) )
					{
						i2c_status |= STATUS_CML_DATA_FAULT;
					}
				}
			}
			else if ( IS_BLOCKCMD && (1 == i2c_rxIdx) )
			{
				// Block length
				if(data > i2c_registerMap[i2c_registerMapIndex].writeBytes)
				{
					i2c_status |= STATUS_CML_DATA_FAULT;
				}
				writeBytes = data + 1;  // Save length from block write command, add one to account for length byte
				i2c_calculatePec(data);
			}
			else
			{
				// Write data to command
				if( (0x00 == i2c_registerMap[i2c_registerMapIndex].cmdCode) && (data >= I2C_NUMPAGES) && (data < 0xFF) )
				{
					// Special handling of a write to PAGE with an illegal value but allows 0xFF
					i2c_status |= STATUS_CML_DATA_FAULT;
				}
				else if( IS_PAGED && (0xFF == I2C_PAGE[0]) )
				{
					// Handles writes of data to paged registers with PAGE = 0xFF (illegal)
					i2c_status |= STATUS_CML_DATA_FAULT;
				}
				else
				{
					// If we got here, everything is good.  Write the value! (to the buffer)
					i2c_buffer[i2c_rxIdx-(IS_BLOCKCMD?2:1)] = data;
					i2c_calculatePec(data);
				}
			}
			i2c_rxIdx++;

			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
			break;

		case I2C_SRX_STOP_RESTART:       // A STOP condition or repeated START condition has been received while still addressed as Slave    
                                                        // Enter not addressed mode and listen to address match
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);  // Enable TWI-interface and release TWI pins
			i2c_busy = 0;  // We are waiting for a new address match, so we are not busy

			if(!(i2c_state & I2C_STATE_ERROR))
			{
				// Done writing data.  Do something if no error since last SLA+W.
				if( (0 == i2c_registerMap[i2c_registerMapIndex].readBytes) && (0 == i2c_registerMap[i2c_registerMapIndex].writeBytes) )
				{
					// Send byte command
					i2cCmdQueuePush(&i2c_command);
				}
				else if(i2c_rxIdx > writeBytes)
				{
					// We received at least the correct amount of data (extra beyond PEC gets flagged as error), so write to actual register
					uint8_t i;
					uint16_t pageOffset;  // 16 bits to handle word writes with page > 127
					pageOffset = (IS_PAGED ? (I2C_PAGE[0] * (i2c_registerMap[i2c_registerMapIndex].writeBytes + (IS_LBLOCK?1:0))) : 0);
					if(IS_LBLOCK)
					{
						*(i2c_registerMap[i2c_registerMapIndex].ramAddr + pageOffset) = writeBytes - 1;  // Store length in first byte
					}
					for(i=0; i<(IS_BLOCKCMD?writeBytes-1:writeBytes); i++)  // Copy only the number of bytes written.  Subtract one for length byte in block commands
					{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
						*(i2c_registerMap[i2c_registerMapIndex].ramAddr + pageOffset + (IS_LBLOCK?1:0) + i) = i2c_buffer[i];
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
						// Unmangle byte order since SMBus sends lowest byte first - Write from end to the beginning
						*(i2c_registerMap[i2c_registerMapIndex].ramAddr + pageOffset + i2c_registerMap[i2c_registerMapIndex].writeBytes + (IS_LBLOCK?1:0) - 1 - i) = i2c_buffer[i];
#endif
					}
				}
			}
			break;

		case I2C_SRX_ADR_DATA_NACK:      // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
		case I2C_SRX_GEN_DATA_NACK:      // Previously addressed with general call; data has been received; NOT ACK has been returned
		case I2C_STX_DATA_ACK_LAST_BYTE: // Last data byte in TWDR has been transmitted (TWEA = \930\94); ACK has been received
//		case I2C_NO_STATE              // No relevant state information available; TWINT = \930\94
		case I2C_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
			TWCR |= _BV(TWSTO) | _BV(TWINT); //Recover from I2C_BUS_ERROR, this will release the SDA and SCL pins thus enabling other devices to use the bus
			break;

		default:     
			TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
      
			i2c_busy = 0; // Unknown status, so we wait for a new address match that might be something we can handle
			break;
	}
	
	if(i2c_status)
	{
		// We just set status so update it
		I2C_STATUS_CML[0] |= i2c_status;
#ifdef I2C_ENABLE_STATUS_WORD
		uint8_t i;
		for(i=0; i<I2C_NUMPAGES; i++)
		{
			I2C_STATUS_WORD[i] |= STATUS_WORD_CML;
		}
#endif
		i2c_state |= I2C_STATE_ERROR;
	}
}


