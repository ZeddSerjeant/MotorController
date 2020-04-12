// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Detect (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)


#include "head.h"

// for timing smaller than a single time loop.
#define TIMER0_INITIAL 118

#define LED_PERIOD 500
unsigned short int led_duty_cycle = 0; // Duty cycle of LED as on_time[ms]
unsigned short int led_duty_cycle_counter = 0;

volatile union
{
    unsigned char byte;
    struct
    {
        unsigned RA0:1;
        unsigned RA1:1;
        unsigned RA2:1;
        unsigned RA3:1;
        unsigned RA4:1;
        unsigned RA5:1;
    } bits;
    struct
    {
        unsigned DAT_LED:1;
        unsigned CLK_LED:1;
        unsigned IND_LED:1;
        unsigned _:3;
    };
} PORTA_SH;


void __interrupt() ISR()
{
    if (TIMER0_INTERRUPT_FLAG) // if the timer0 interrupt flag was set (timer0 triggered)
    {
        TIMER0_INTERRUPT_FLAG = CLEAR; // clear interrupt flag since we are dealing with it
        TIMER0_COUNTER = TIMER0_INITIAL + 2; // reset counter, but also add 2 since it takes 2 clock cycles to get going
        // move counters, which is the job of this timer interrupt
        led_duty_cycle_counter++; // increment the led counter
        
        if (led_duty_cycle_counter >= led_duty_cycle)
        {
            if (led_duty_cycle_counter >= LED_PERIOD)
            {
                led_duty_cycle_counter -= LED_PERIOD; //reset led counter safely
                // led_state = ON; // we are in the ON part of the duty cycle
            }
            else
            {
                PORTA_SH.IND_LED = OFF;
            }
        }
        else
        {
            PORTA_SH.IND_LED = ON; // within On part of duty cycle
        }
        
        // IND_LED = IND_led_state;
        // LED = led_test_state; //TTT
        // IND_LED = ON;
    }
}

void main() {
    //set up IO
    DAT_LED_TYPE = DIGITAL;
    DAT_LED_PIN = OUTPUT;

    CLK_LED_TYPE = DIGITAL;
    CLK_LED_PIN = OUTPUT;

    IND_LED_TYPE = DIGITAL;
    IND_LED_PIN = OUTPUT;

    PORTA_SH.DAT_LED = ON;
    PORTA_SH.CLK_LED = ON;

    // Set up timer0
    // calculate intial for accurate timing $ inital = TimerMax-((Delay*Fosc)/(Prescaler*4))
    TIMER0_COUNTER = TIMER0_INITIAL; // set counter
    TIMER0_CLOCK_SCOURCE = INTERNAL; // internal clock
    PRESCALER = 0; // enable prescaler for Timer0
    PS2=0; PS1=1; PS0=0; // Set prescaler to 1:8
    TIMER0_INTERRUPT = ON; // enable timer0 interrupts

    //setup flashing led
    led_duty_cycle = 250; //[ms]

    //turn on interrupts
    GLOBAL_INTERRUPTS = ON;

    while (1)
    {
        PORTA = PORTA_SH.byte;
    }

    return;
}
