/*
 * Ibus.c
 *
 * Created: July 2019
 *  Author: Doug Hadfield
 * Reads I-Bus protocol from UART0 RX
 * Loads 14 RC channel values into array
 * RC values are 16 bit 1000-2000
*/

#include "Doug.h"
#include "Ibus.h"

uchar tx_in_ptr;
volatile uchar tx_out_ptr;
uchar tx_buffer[TXBUFSIZ];
volatile uchar tx_buf_remain;

/* RX private variables */

enum {GET_LENGTH, GET_DATA, GET_CHKSUML, GET_CHKSUMH, DISCARD} state;   // state machine for protocol in interrupt routine

uint8_t RxPtr;                                  // pointer in buffer
uint8_t RxPacketLen;                            // message length
uint16_t RxChecksum;                            // checksum calculation
uint8_t RxChecksumL;                            // checksum lower byte received
uint8_t sensorType[SENSORMAX];                  // sensor types for defined sensors
uint16_t sensorValue[SENSORMAX];                // sensor data for defined sensors
uint8_t NumberSensors;                          // number of sensors

volatile uint8_t RxBuf[PROTOCOL_LENGTH];        // message buffer
volatile uint16_t IbusChannels[PROTOCOL_CHANNELS]; // servo data received
volatile uint8_t failsafe;                      // failsafe counter - indicates when RSSI is lost

/*
* hardware UART peripheral setup
*/
void RC_IBus_Init(void)
{
    uint8_t count;                          // counter for array clear loop

	// set up UART0
#define MYUBRR ((FOSC)/8/(BAUD)-1)          //	Baud rate register calculation
	UBRR0H = (uchar)(MYUBRR>>8);            // set baud rate to 115200 for i-bus
	UBRR0L = (uchar)MYUBRR;
    UCSR0A |= 0b00000010;                   // set U2X0 bit for double speed baud rate
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);         // UCSR0C defaults to 8 bits no parity one stop bit

    // set up TX channel to PC monitor
	tx_buf_remain=tx_in_ptr=tx_out_ptr=0;	// initialise all write buffer pointers to zero
	cbi(UCSR0B,UDRIE0);						// clear interrupt for TX (nothing to write yet)

    // set up timer and RX channel for I-Bus reception
    // timer 0 used to measure start of i-Bus data stream
    TCCR0B = 0b00000101;                    // set Tmr0 clock to prescaler / 1024 (approx 16 counts per ms)
    TCNT0 = 0;                              // zero counter to begin with
    state = DISCARD;                        // initialise state machine to ignore existing stream
    
    DDRB |= (1<<DDB5);                      // set L LED port to output (RX signal indicator)
    PORTB &= 0b11011111;                    // make sure LED is off initially

    for (count=0;count<PROTOCOL_CHANNELS;count++)
        IbusChannels[count] = 1500;         // centre servos when first powered up
    IbusChannels[THROTTLE] = 1000;          // zero throttle channel (usually channel 3)
#ifdef DUAL_MOTOR                           // if we have two throttle channels
    IbusChannels[THROTTLE2] = 1000;         // use two channels for throttle
#endif // DUAL_MOTOR
	sbi(UCSR0B,RXCIE0);						// enable read interrupts (start reading characters)
}


void Write(char OutByte)
{
	while(tx_buf_remain>=TXBUFSIZ)	// no space in buffer
	;							// spin waiting (interrupts must be turned on at this point
	
	cbi(UCSR0B,UDRIE0);					// disable tx interrupts while manipulating buffer
	tx_buffer[tx_in_ptr++] = OutByte;	// put data in output buffer
	if (tx_in_ptr >=TXBUFSIZ)
	tx_in_ptr=0;					// wrap ptr when it exceeds buffer
	tx_buf_remain++;					// increment remaining data
	sbi(UCSR0B,UDRIE0);					// enable tx interrupts
}

void WriteLine(const char *writebuf)
{
	uchar ptr = 0;
	
	while(writebuf[ptr] != 0)		// until end of string
	{
		Write(writebuf[ptr++]);		// output character to UART
	}
}


// Interrupt handling routines

/* USART WRITE */
ISR(USART_UDRE_vect)		// UART TX buffer is empty and data is waiting
{
	UDR0=tx_buffer[tx_out_ptr++];			// place char from buffer into tx reg - this clears int condition
	if (tx_out_ptr>=TXBUFSIZ)
        tx_out_ptr=0;					// wrap ptr if exceeds buffer
	tx_buf_remain--;					// decrement as we remove chars from buffer
	if(!tx_buf_remain)					// no more characters in write buffer
	cbi(UCSR0B,UDRIE0);				// so disable tx interrupts
}

/*
 * USART READ
 * implement I-Bus protocol
 * WAIT - wait for gap of more than 3ms (48 counts of timer0 prescaled 1024
 * LENGTH - first packet byte is packet length (usually 0x20 - servo data)
 * COMMAND - type of packet - 0x40 for servo data
 * DATA - "LENGTH" btyes, including overhead (two command and two checksum bytes)
 * CHECKSUM - two byte checksum, 0xFFFF - all above bytes (including first two)
 */

ISR (USART_RX_vect)                 // UART has received a character
{
    if (TCNT0 >= PROTOCOL_TIMEGAP)  // We have seen the correct pause in transmission
    {                               //        inter-char gap is 125us, so anything greater than 1ms is good
      state = GET_LENGTH;           // we are at start of new packet
    }
    TCNT0 = 0;                      // clear timer at each read character  
    uint8_t RxData = UDR0;           // read character from UART
    switch (state)
    {
      case GET_LENGTH:
        if (RxData <= PROTOCOL_LENGTH && RxData > PROTOCOL_OVERHEAD)    // length value is sensible
        {
          RxPtr = 0;                          // point to start of read buffer, as we at at start of packet
          RxPacketLen = RxData - PROTOCOL_OVERHEAD; // expected length of packet data
          RxChecksum = 0xFFFF - RxData;         // start calculating checksum
          state = GET_DATA;                 // we're now in data part of packet
        }
        else                                // length value is not sensible
        {
          state = DISCARD;
        }
        break;

      case GET_DATA:                        // we're in data part of packet
        RxBuf[RxPtr++] = RxData;             // read data into RX buffer
        RxChecksum -= RxData;                   //continue calculating checksum
        if (RxPtr == RxPacketLen)                     // have we reached end of data
        {
          state = GET_CHKSUML;              // we're now in checksum portion of packet
        }
        break;
        
      case GET_CHKSUML:                     // this value is checksum low byte
        RxChecksumL = RxData;
        state = GET_CHKSUMH;                // next value will be checksum high byte
        break;

      case GET_CHKSUMH:                     // this value is checksum high byte - end of packet
        // Validate checksum
        if (RxChecksum == ((RxData << 8) + RxChecksumL))    // does our calculated checksum agree with read value
        {                                           // Checksum is all fine Execute command - 
            failsafe=0;                             // reset the failsafe counter as we're receiving signal
            IBUS_ALARM_OFF;                         // turn off failsafe alarm now we're receiving data
            if (RxBuf[0]==PROTOCOL_COMMAND40)       // Valid servo command received - extract channel data
            {
                for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)  // save channel data in 16 bit words
                {
                    IbusChannels[ i/2 ] = RxBuf[i] | (RxBuf[i + 1] << 8); // data is little-endian, so second byte is high 
                }
            }
            sbi(PORTB,5);                           // turn on L LED to indicate we're receiving valid data...
                                                    //  ...this will occur every 7ms on a good link
                                                    //  LED is turned off every 50hz by PWM interrupt
#ifdef NEVER   // TELEMETRY TRANSMIT SECTION      
          else if (adr<=NumberSensors && adr>0 && RxPacketLen==1)
          {
            // all sensor data commands go here
            // we only process the len==1 commands (=message length is 4 bytes incl overhead) to prevent the case the
            // return messages from the UART TX port loop back to the RX port and are processed again. This is extra
            // precaution as it will also be prevented by the PROTOCOL_TIMEGAP required
//            delayMicroseconds(100);
            switch (RxBuf[0] & 0x0f0) {
              case PROTOCOL_COMMAND_DISCOVER: // 0x80, discover sensor
                cnt_poll++;  
                // echo discover command: 0x04, 0x81, 0x7A, 0xFF 
                stream->write(0x04);
                stream->write(PROTOCOL_COMMAND_DISCOVER + adr);
                RxChecksum = 0xFFFF - (0x04 + PROTOCOL_COMMAND_DISCOVER + adr);
                break;
              case PROTOCOL_COMMAND_TYPE: // 0x90, send sensor type
                // echo sensortype command: 0x06 0x91 0x00 0x02 0x66 0xFF 
                stream->write(0x06);
                stream->write(PROTOCOL_COMMAND_TYPE + adr);
                stream->write(sensorType[adr]);
                stream->write(0x02); // always this value - unkwown
                RxChecksum = 0xFFFF - (0x06 + PROTOCOL_COMMAND_TYPE + adr + sensorType[adr] + 2);
                break;
              case PROTOCOL_COMMAND_VALUE: // 0xA0, send sensor data
                cnt_sensor++;
                // echo sensor value command: 0x06 0x91 0x00 0x02 0x66 0xFF 
                stream->write(0x06);
                stream->write(PROTOCOL_COMMAND_VALUE + adr);
                stream->write(sensorValue[adr] & 0x0ff);
                stream->write(sensorValue[adr] >> 8); 
                RxChecksum = 0xFFFF - (0x06 + PROTOCOL_COMMAND_VALUE + adr + (sensorValue[adr]>>8) + (sensorValue[adr]&0x0ff));
                break;
              default:
                adr=0; // unknown command, prevent sending chksum
                break;
            }
            if (adr>0) {
              stream->write(RxChecksum & 0x0ff);
              stream->write(RxChecksum >> 8);              
            }
          }
#endif // PROTOCOL
        }
        state = DISCARD;
        break;

      case DISCARD:
      default:
        break;
    }
}
