/*
 * RC_PWM.h
 *
 * Created: 07/2019
 *  Author: Doug
 * 
 * Implements servo PWM outputs for first 10 I-Bus channels, plus alarm on pin A3
 * Conditional code for output of ESC pulses for two brushed motors on A4 and A5
 * 
 * Alarm beeper triggered from PWM 20ms period in interrupt routine
 * 
 */ 


#ifndef RC_PWM_H_
#define RC_PWM_H_
/*
 * CONDITIONAL COMPILES
 */
#define PWM_ESC                     // enable brushed ESC functionality

extern void RC_Pwm_Init(void);      // hardware initialisation for PWM generator
extern uint8_t RC_MsClock;          // counter incremented every PWM interrupt - every 2ms
extern uint8_t AlarmFlag;           // flags to turn on certain alarms

#define THR_ALARM 0
#define BATT_ALARM 1
#define IBUS_ALARM 2
#define SW_ALARM 3

#define THR_ALARM_ON sbi(AlarmFlag,THR_ALARM)
#define BATT_ALARM_ON sbi(AlarmFlag,BATT_ALARM)
#define IBUS_ALARM_ON sbi(AlarmFlag,IBUS_ALARM)
#define SW_ALARM_ON sbi(AlarmFlag,SW_ALARM)

#define THR_ALARM_OFF cbi(AlarmFlag,THR_ALARM)
#define BATT_ALARM_OFF cbi(AlarmFlag,BATT_ALARM)
#define IBUS_ALARM_OFF cbi(AlarmFlag,IBUS_ALARM)
#define SW_ALARM_OFF cbi(AlarmFlag,SW_ALARM)

#define BEEPERON sbi(PORTC,3)       // Pin A3 is beeper 
#define BEEPEROFF cbi(PORTC,3)      // Pin A3 is beeper

#ifdef PWM_ESC                      // if we've enabled brushed ESC functionality in header
#define DUAL_MOTOR                  // ensure both throttle channels fail-safe on signal loss
#define ESCLEFT 2                   // RC channel for left motor throttle
#define ESCRIGHT 3                  // RC channel for right motor throttle
extern uint8_t EscDutyCycle;        // duty cycle for ESC pulses
#endif // PWM_ESC

#endif /* RC_PWM_H_ */
