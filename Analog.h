/*
    Analog.h

    Created: 06/12/2015 20:03:00
    Author: Doug Hadfield

 	Initialise and operate the Atmega ADC unit

*/


#ifndef Analog_H_
#define Analog_H_

#include <avr/io.h>	// needs to be included once for every c source file
#include <avr/sfr_defs.h>	// bit manipulation macros
#include <avr/interrupt.h>	// defines interrupt routines etc.

extern void ADC_Init(void); // initialise ADC hardware
extern uint16_t ADC_Read(uint8_t channel);  // 10 bit conversion
#define ADC_Read8(channel) (ADC_Read(channel)>>2)   // 8 bit version of ADC_Read

#endif /* Analog_H_ */
