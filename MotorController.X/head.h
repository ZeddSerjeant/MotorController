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

//TIMER0
#define PRESCALER PSA
#define TIMER0_COUNTER TMR0
#define TIMER0_CLOCK_SCOURCE T0CS
#define TIMER0_INTERRUPT T0IE
#define TIMER0_INTERRUPT_FLAG TMR0IF

