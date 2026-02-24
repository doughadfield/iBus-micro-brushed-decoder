/*
 * Doug.h
 *
 * Created: 06/12/2015 20:03:00
 *  Author: doug
 *
 *	Contains Doug's defines and macros
 * 
 */ 


#ifndef Doug_H_
#define Doug_H_

// #define USE_TWI		// use Two Wire Peripheral for display output

#include <avr/io.h>	// needs to be included once for every c source file
#include <avr/sfr_defs.h>	// bit manipulation macros
#include <avr/interrupt.h>	// defines interrupt routines etc.

// Doug's global definitions, for all files


typedef unsigned char uchar;
typedef volatile unsigned char vuchar;
#define NOP()  __asm__ __volatile__ ("nop")	// delay for 62.5ns at 16MHz clock (one instruction)
#define sbi(reg,bit) ((reg) |= _BV(bit))	//  bit set macro
#define cbi(reg,bit) ((reg) &= ~(_BV(bit)))	//  bit clear macro

// delay 20.4ms per 65535 count (at 16Mhz clock)
// delay  310us per 1000 count (at 16Mhz clock)
// delay   32us per 100 count
// delay  4.1us per 10 count
// delay  2.4us per 5 count
// delay  1.2us per 1 count
// delay    1us per 0 count
// delay formula = (approx) (0.31*count)+1 MicroSeconds
extern void delay(uint16_t count);		// delay for short time
extern void LongDelay(uint16_t count);		// double loop with delay()

#define OUTSTRLEN 10				// length of longest itoa() conversion (binary byte) plus NULL
extern char outstr[OUTSTRLEN];			// string to hold ascii conversions

/* Doug's implementation of itoa()
* returns RIGHT JUSTIFIED FIXED LENGTH string of up to 9 chars (plus null)
* with converted characters in the rightmost locations
* leading space is filled with ' ' (0x20) character
* Field is field width (number of printed characters) of returned string
* base is number base (2,10,16) of printed ascii representation of num
*/
extern char* itoa(uint16_t num, uchar base, uchar field);

// convenient cases of itoa, passing only one argument
#define ByteToBin(byte)	itoa((uint16_t)(byte),2,9)		// binary byte, 9 characters (1 leading space)
#define ByteToDec(byte)	itoa((uint16_t)(byte),10,3)		// Decimal byte, 3 characters (max. 255)
#define ByteToHex(byte)	itoa((uint16_t)(byte),16,2)		// Hex byte, 2 characters (max. FF)
#define IntToHex(word)	itoa((word),16,4)			// Hex 16-bit int, 4 characters (max. FFFF)
#define IntToDec(word)	itoa((word),10,5)			// Decimal 16-bit int, 5 characters (max. 65535)

#endif /* Doug_H_ */