
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 * 
 * implements I-Bus protocol read on the UART RX input
 * 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef IBUS_H
#define	IBUS_H

#include "Doug.h"
#include "RC_PWM.h"                     // for DUAL_MOTOR setting

#define FOSC 16000000					// Oscillator frequency
#define BAUD 115200						// Baud Rate for serial comms

// calling interface to UART I/O
extern void Write(char);
extern void WriteLine(const char *);    // Write a character to the UART
extern void RC_IBus_Init(void);	// Initialise the UART and Timer 0

// variables for interrupt driven write buffer
#define TXBUFSIZ 16
extern uchar tx_in_ptr;
extern volatile uchar tx_out_ptr;
extern uchar tx_buffer[];
extern volatile uchar tx_buf_remain;

// variables for interrupt driven read buffer

#define PROTOCOL_LENGTH 0x20
#define PROTOCOL_OVERHEAD 3     // packet is <len><cmd><data....><chkl><chkh>, overhead=cmd+chk bytes
#define PROTOCOL_TIMEGAP 35     // Packets are received very ~7ms so use ~half that for the gap
#define PROTOCOL_CHANNELS 14            // standard I-Bus outputs 14 channels of servo data
#define PROTOCOL_COMMAND40 0x40         // Command to set servo or motor speed is always 0x40
#define PROTOCOL_COMMAND_DISCOVER 0x80  // Command discover sensor (lowest 4 bits are sensor)
#define PROTOCOL_COMMAND_TYPE 0x90      // Command discover sensor (lowest 4 bits are sensor)
#define PROTOCOL_COMMAND_VALUE 0xA0     // Command send sensor data (lowest 4 bits are sensor)
#define SENSORMAX 10                    // Max number of sensors

#define THROTTLE 2                      // channel number for throttle (counting from 0)
#ifdef DUAL_MOTOR                       // "DUAL_MOTOR" defined in RC_PWM.h
#define THROTTLE2 3                     // second throttle channel if using dual motors
#endif // DUAL_MOTOR

extern volatile uint8_t ExtLen;         // length of packet 
extern volatile uint16_t IbusChannels[PROTOCOL_CHANNELS]; // store for servo data received

extern volatile uint8_t failsafe;       // failsafe counter - indicates when RSSI is lost

#endif	/* IBUS_H */

