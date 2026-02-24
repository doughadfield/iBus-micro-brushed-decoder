/*
 * Doug.c
 *
 * Created: 04/01/2016 12:33:28
 *  Author: hadfiedo
 *
 * Contains general purpose routines (delays etc)
 *
 */ 

#include "Doug.h"

// delay 20.4ms per 65535 count (at 16Mhz clock)
// delay  310us per 1000 count (at 16Mhz clock)
// delay   32us per 100 count
// delay  4.1us per 10 count
// delay  2.4us per 5 count
// delay  1.2us per 1 count
// delay    1us per 0 count
// delay formula = (approx) (0.31*count)+1 MicroSeconds
// so counts = (<delay in us> - 1)/0.31
void delay(uint16_t count)
{
	while(count-- > 0)
		NOP();
}

void LongDelay(uint16_t count)		// double loop, with delay
{
	while(count-- > 0)
		delay(0xFFFF);
}

char outstr[OUTSTRLEN];					// string to hold ascii conversions


/* 
* Implementation of itoa()
* returns right justified string, of length 9 (plus NULL)
*/
char* itoa(uint16_t num, uchar base, uchar field)
{
	uchar i, rem;
	
	if(field>=OUTSTRLEN)						// field specifier is longer than string
	{
		outstr[0] = '\0';
		return outstr;							// return blank string as error condition
	}
	for(i=0;i<OUTSTRLEN-1;i++)
		outstr[i] = '0';						// fill string with blanks to start with
	outstr[i] = '\0';							// terminate string with NULL
	
	/* Handle 0 explicitly, otherwise empty string is printed for 0 */
	if (num == 0)
	{
		outstr[--i] = '0';
			return (outstr+(OUTSTRLEN-1-field));
	}
	
	// Process individual digits
	while (num != 0)
	{
		rem = num % base;
		outstr[--i] = (rem > 9)? (rem-10) + 'A' : rem + '0';
		num = num/base;
	}
	
	return (outstr+(OUTSTRLEN-1-field));
}

