/*
    Analog.c

    Created: 04/01/2016 12:33:28
    Author: Doug Hadfield

    Initialise and operate the Atmega ADC unit

*/
#include "Doug.h"
#include "Analog.h" // contains arduino as well as local defines

void ADC_Init(void) // initialise ADC hardware
{
	ADMUX = 0b11000111; // VRef selected as internal 1.1v reference
                        // Bit 5 = 0 to RIGHT justify result (16bit result value)
                        // select Analog Pin 7 as source by default

	ADCSRA = 0b10000111;    // enable ADC unit, prescaler on low 3 bits
	// 0b0100 = clock / 16 = 1MHz - accuate to 7 bits
	// 0b0101 = clock / 32 = 500kHz - accurate to 8 bits
	// 0b0110 = clock / 64 = 250kHz - fastest with 12 bit accuracy
	// 0b0111 = clock / 128 = 125kHz
}

// receive analog pin to convert from
// routine initiates an ADC conversion, waits for the result
// then passes it back. Result is 10 bit resolution in 16 bit word

uint16_t ADC_Read(uint8_t channel)  // pass analog pin for conversion
{
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);      // set input channel to passed pin
	ADCSRA |= (1 << ADSC);          // start ADC conversion
	while (ADCSRA & (1 << ADSC))    // when ADSC bit clears, conversion is complete
		;
	return ADC;                     // return conversion result as 16 bit word
}
