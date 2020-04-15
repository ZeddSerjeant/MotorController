#include <xc.h> // include processor files - each processor file is guarded. 

enum FLAGS {OFF=0, OUTPUT=0, INTERNAL=0, CLEAR=0, DIGITAL=0, ANALOG=1, INPUT=1, ON=1};
#define GLOBAL_INTERRUPTS GIE

//Pins

#define DAT_LED RA0
#define DAT_LED_PIN TRISA0 
#define DAT_LED_TYPE ANS0
#define DAT_LED_BIT 0b000001

#define CLK_LED RA1
#define CLK_LED_PIN TRISA1
#define CLK_LED_TYPE ANS1
#define CLK_LED_BIT 0b000010

#define IND_LED RA2
#define IND_LED_PIN TRISA2 
#define IND_LED_TYPE ANS2
#define IND_LED_BIT 0b000100

#define PWM_MOTOR RC5
#define PWM_MOTOR_PIN TRISC5

//TIMER0
#define PRESCALER PSA
#define TIMER0_COUNTER TMR0
#define TIMER0_CLOCK_SCOURCE T0CS
#define TIMER0_INTERRUPT T0IE
#define TIMER0_INTERRUPT_FLAG TMR0IF

//PWM
//parts are specified as an offset within a register
#define PWM_CONTROL CCP1CON
// #define PWM_MODE P1M
#define PWM_MODE (unsigned char)6
// #define PWM_OUTPUT CCP1M
#define PWM_OUTPUT (unsigned char)0
#define PWM_DUTYCYCLE_MSB CCPR1L
// #define PWM_DUTYCYCLE_LSB DC1B
#define PWM_DUTYCYCLE_LSB (unsigned char)4
#define PWM_PERIOD PR2

//timer 2 (needed for PWM)
#define TIMER2_CONTROL T2CON
#define TIMER2_ON 2
#define TIMER_CLOCK_PRESCALE (unsigned char)0
#define TIMER_CLOCK_POSTSCALE (unsigned char)3
#define PRESCALE_1 (unsigned char)0b00
#define PRESCALE_4 (unsigned char)0b01
#define PRESCALE_16 (unsigned char)0b10
 
// #define TIMER2_PRESCALER T2CKPS

#define ACTIVE_HIGH_ACTIVE_HIGH (unsigned char)0b1100
#define SINGLE_OUTPUT (unsigned char)0b00
