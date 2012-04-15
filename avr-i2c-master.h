/*************************************************************************
Title:    MRBus AVR I2C Library
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     avr-i2c-master.h
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

#ifndef _AVR_I2C_H
#define _AVR_I2C_H

// Application Specific Defines - these may need to be adjusted

#define I2C_MAX_BUFFER_SIZE 8   // Set this to the largest message size that will be sent including address byte.
#define I2C_TWBR            0x12        // I2C Bit rate Register setting.
                                        // Se Application note for detailed 
                                        // information on setting this value.

#define I2C_vect            TWI_vect // This should match the ISR vector define of the part you're using

extern uint8_t i2c_buffer[ I2C_MAX_BUFFER_SIZE ];    // Transceiver buffer
extern uint8_t i2c_bufferLen;                   // Number of bytes to be transmitted.
extern uint8_t i2c_bufferIdx;
extern uint8_t i2c_state;      // State byte. Default set to I2C_NO_STATE.


// IF TWI Interrupt is enabled then the Transceiver is busy

void i2c_master_init(void);
uint8_t i2c_busy(void);
void i2c_transmit(uint8_t *msgBuffer, uint8_t msgLen, uint8_t omitStop);
uint8_t i2c_receive(uint8_t *msgBuffer, uint8_t msgLen);

#define I2C_MSG_RECV_GOOD     0       // i2c_status, bit 0 shows last message is good
#define I2C_MSG_OMIT_STOP     1       // i2c_status, omit stop at the end of transmit
#define I2C_READ_BIT          0       // Bit 0 is Read / !Write in address

/****************************************************************************
  I2C State codes
****************************************************************************/
// General I2C Master staus codes                      
#define I2C_START                  0x08  // START has been transmitted  
#define I2C_REP_START              0x10  // Repeated START has been transmitted
#define I2C_ARB_LOST               0x38  // Arbitration lost

// I2C Master Transmitter staus codes                      
#define I2C_MTX_ADR_ACK            0x18  // SLA+W has been tramsmitted and ACK received
#define I2C_MTX_ADR_NACK           0x20  // SLA+W has been tramsmitted and NACK received 
#define I2C_MTX_DATA_ACK           0x28  // Data byte has been tramsmitted and ACK received
#define I2C_MTX_DATA_NACK          0x30  // Data byte has been tramsmitted and NACK received 

// I2C Master Receiver staus codes  
#define I2C_MRX_ADR_ACK            0x40  // SLA+R has been tramsmitted and ACK received
#define I2C_MRX_ADR_NACK           0x48  // SLA+R has been tramsmitted and NACK received
#define I2C_MRX_DATA_ACK           0x50  // Data byte has been received and ACK tramsmitted
#define I2C_MRX_DATA_NACK          0x58  // Data byte has been received and NACK tramsmitted

// I2C Slave Transmitter staus codes
#define I2C_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define I2C_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define I2C_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define I2C_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define I2C_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = 0); ACK has been received

// I2C Slave Receiver staus codes
#define I2C_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define I2C_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define I2C_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define I2C_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define I2C_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define I2C_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define I2C_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define I2C_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define I2C_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// I2C Miscellaneous status codes
#define I2C_NO_STATE               0xF8  // No relevant state information available
#define I2C_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition


#endif

