/* 
 * File:   main.c
 * Author: Doug Hadfield
 *
 * Created on 09 August 2019, 15:02
 *
 * Arduino (nano, pro mini) code for I-Bus decode into:
 *
 * channels 1-10 - servo outputs
 * channels 11-14 - switch outputs
 * Servo outputs on Arduino ports D2-D11
 * switch outputs on Arduino ports A0-A2
 * BEEPER alarm on Arduino port A3 (PORTC,3)
 * 
 * Arduino port D12 is jumper input for servo test mode (insert bind jumper before power-on)
 * Arduino port D13 is LED L - indicates valid data being received from I-Bus feed
 *
 * L LED = RX signal
 * servo centre on power up (except "THROTTLE" and optionally "THROTTLE2")
 * Throttle zero on power up (THROTTLE(and THROTTLE2) defined in ibus.h)
 * failsafe - throttle(s) to zero 1/2 second after RX signal loss
 * 
 * Optional brushed motor ESC output on pins A4 and A5 - defined in RC_PWM.h
 * 
 * Adding a jumper to Arduino pin D12 enables servo test mode, until first I-Bus packet
 * 
 *     
 * Battery Voltage Monitor - settings for alarm threshold and motor cutoff threshold
 * With 3k3 and 470ohm voltage divider and internal vref
 * Ratio of voltage to reading is 0.853 (voltage / 853) * 1000 = reading
 * readings are:
 * 
 * 8.7v = 1020 (Max voltage for 2C battery)
 * 6.6v = 774
 * 6.5v = 762
 * 6.4v = 750
 * 6.3v = 739
 * 6.2v = 727
 * 6.1v = 715
 * 6.0v = 704
 * 
 * 4.4v = 516 (Max voltage for 1C battery)
 * 3.5v = 410
 * 3.4v = 399
 * 3.3v = 387
 * 3.2v = 375
 * 3.1v = 363
 * 3.0v = 352
 * 
 */


#include "Doug.h"
#include "Ibus.h"
#include "RC_PWM.h"
#include "Analog.h"

extern void PrintPacket(void);
extern void ServoSweep(void);

/*
 * Main thread takes care of:
 *  throttle monitoring on power-up
 *  continuous battery monitoring with alarm and throttle kill
 *  Servo sweep testing (with jumper link)
 *
 * Alarms (beeper) for:
 *      Throttle high at start
 *      Battery Low
 *      i-bus signal lost
 *      alarm switch (usually channel 14)
 * 
 *i-Bus input, servo PPM and ESC outputs are all interrupt driven
 * 
 */
int main()
{
    uint16_t VBatt;                     // battery voltage reading from ADC

    // settings for servo test pin, switch and beeper outputs
    cbi (DDRB,4);                       // make sure pin B4 is input
    sbi(PORTB,4);                       // set bit 4 high, to enable pull-up on input 
    DDRC = 0b00001111;                  // set Port C pins 0-3 to OUTPUT (A0 - A3)
    
    ADC_Init();                         // set up ADC unit for measuring battery voltage
    RC_IBus_Init();                     // initialise UART and Timer0
    RC_Pwm_Init();                      // initialise hardware for PWM and ESC outputs

    sei();                              // Globally enable interrupts, to enable ibus rx, PWM output

    VBatt = ADC_Read(7);                // read battery voltage from ADC
      
#ifdef PWM_ESC                          // if we're compiling for ESC 
    while(VBatt > 1024)                 // battery voltage greater than 8.8v, so higher than 2C
    {
        BEEPERON;                       // turn on continuous alarm until battery condition removed
        LongDelay(20);                  // delay in case we have debug messages to read
    }

    if(VBatt > 600)                     // battery is greater than 1C, so must be 2C
        EscDutyCycle = 2;               // 1 in 3 dutycycle for 2C batteries
    else
        EscDutyCycle = 0;               // 1 in 1 duty cycle for 1C battery
#endif // PWM_ESC            

    if(bit_is_clear(PINB,4))            // Is jumper connector on pin PB4 - pull-up will keep it high otherwise
    {
        ServoSweep();                   // Test servos by sweeping through full motion range
    }

    while (bit_is_clear(PORTB,5))       // no i-bus data yet received
    {
        IBUS_ALARM_ON;                  // so spin waiting for RX to come up, with alarm sounding
    }
    IBUS_ALARM_OFF;                     // we now have valid i-bus signal
    while(IbusChannels[THROTTLE]>1003)  // throttle stick is NOT at min
    {
        cbi(TIMSK2,0);                  // Turn off timer 2 overflow INT so ESC pulses do not start
        THR_ALARM_ON;                   // alert that throttle is up
    }
    sbi(TIMSK2,0);                      // turn on motor ESC interrupts
    THR_ALARM_OFF;                      // disable throttle warning beeper

    while(1)                            // main processing loop
    {            
        // Battery voltage warning section
            
        VBatt = ADC_Read(7);            // read battery voltage from ADC pin A7
        if (VBatt > 500)                // we are on a 2C battery
        {
           if(VBatt < 774)              // Batt voltage has dropped below 6.6v
                BATT_ALARM_ON;          // set beeper flag to sound beeper
           else
               BATT_ALARM_OFF;          // reset alarm if battery recovers

#ifdef PWM_ESC           
           if(VBatt < 750)              // voltage below 6.4v (3.2v per cell)
               EscDutyCycle = 5;        // reduce motor power
           if(VBatt < 727)              // battery voltage below 6.1v
               cbi(TIMSK2,0);           // turn off motors altogether
           else
               sbi(TIMSK2,0);           // turn motors back on if battery recovers
#endif // PWM_ESC
        }
        else
        {
           if(VBatt < 410)              // Batt voltage has dropped below 3.4v
                BATT_ALARM_ON;          // set beeper flag to sound beeper
           else
               BATT_ALARM_OFF;          // reset alarm if battery recovers
           
#ifdef PWM_ESC
           if(VBatt < 387)              // voltage below 3.2v
               EscDutyCycle = 5;        // reduce motor power
           if(VBatt < 363)              // battery voltage down to 3.1v
               cbi(TIMSK2,0);           // turn off motors altogether
           else
               sbi(TIMSK2,0);           // turn motors back on if battery recovers
#endif // PWM_ESC
        }
             
        // Output last 4 channels as switch outputs
        // use 1600us as the threshold, so as to default to OFF at centre (1500)
        if(IbusChannels[10] > 1600)     // channel 11 is turned ON
            sbi(PORTC,0);               // so turn on D12 (PB4)
        else
            cbi(PORTC,0);               // otherwise, turn it off

        if(IbusChannels[11] > 1600)     // channel 12 is turned ON
            sbi(PORTC,1);               // so turn on D13 (PB5)
        else
            cbi(PORTC,1);               // otherwise, turn it off

        if(IbusChannels[12] > 1600)     // channel 13 is turned ON
            sbi(PORTC,2);               // so turn on A1 (PC0)
        else
            cbi(PORTC,2);               // otherwise, turn it off

        if(IbusChannels[13] > 1600)     // channel 14 is hard-wired as ALARM
            SW_ALARM_ON;                // so turn on beeper (channel 14 is beeper switch)
        else
            SW_ALARM_OFF;               // otherwise, turn it off
    } 
 }


/*
 * Servo Sweep
 * Sweep all 10 servos through entire motion range
 * to test and exercise connected servos
 * stop test as soon as we start receiving packets from receiver
 */

void ServoSweep(void)
{
    uint16_t sweep;                                     // value to send to servos
    uint8_t count;
    
    while (bit_is_clear(PINB,5))                        // if LED is out, we're not receiving packets
    {
        for(sweep=1000;sweep<2000;sweep++)              // servo values incrementing
        {
            for(count=0;count<PROTOCOL_CHANNELS;count++)    // scan through all channels
                IbusChannels[count] = sweep;                // load sweep value into channel array
            delay(3000);                                   // set sweep speed
        }
        for(sweep=2000;sweep>1000;sweep--)              // servo values decrementing
        {
            for(count=0;count<PROTOCOL_CHANNELS;count++)    // scan through all channels
                IbusChannels[count] = sweep;                // load sweep value into channel array
            delay(3000);                                   // set sweep speed
        }
    }
}


/*
 * print packet to output
 */
void PrintPacket(void)
{
    uchar count = 0;

    WriteLine("\n\rChannels:\n\r");
    for (count=0;count < PROTOCOL_CHANNELS;count++)                   // for every value in buffer
    {
        WriteLine(IntToDec(IbusChannels[count]));          // output two bytes as int word
        Write(' ');                                     // separate each word with space
    }
    WriteLine("\n\r");                                  // new line after packet
}
