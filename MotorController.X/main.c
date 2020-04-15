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

unsigned short int pwm_duty_cycle = 0;//0x3FF; //[ms]
// unsigned short int pwm_period = 500;
unsigned char speed = 50; // 0->100, represents a percentage of max speed
unsigned char in_voltage = 80; // deciVolts, the voltage on the Motor
unsigned short int max_voltage = 466; // 7.6V max voltage the motor is allowed to run at (95% of 8V)
unsigned short int min_voltage = 147; // 2.4V min voltage
#define COUNTDOWN_TIME (unsigned char)10; // [ms] represents the amount of time to check the voltage of the motor. 
unsigned char countdown = COUNTDOWN_TIME;
__bit make_measurement = 0; // indicates whether a measurement needs to be taken

//control variables


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

union reading // a union combining reading space with the magnitude, since once we have a magnitude, we no longer need the readings. 
{
    // arranged this way for memory locations to match
    unsigned long int magnitude;
    struct
    {
        unsigned char reading2_array[2]; // second adc result
        unsigned char reading1_array[2]; // first adc result.
    };
    struct
    {
        unsigned short int reading2; //reading2_array[1] # reading2_array[0]
        unsigned short int reading1; //reading1_array[1] # reading1_array[0]
    };
} sample;
// Save memory!

unsigned short int calcPWM(unsigned char period, unsigned char ratio)
{
    if (ratio == 100)
    {
        return 0x3FF; // full rate
    }
    else if (ratio == 0)
    {
        return 0;
    }
    return (4*((__uint24)period+1)*ratio)/100;
}

// unsigned char calcRatio(unsigned char speed)
// {
//     (((max_voltage - min_voltage)*speed)/(in_voltage))/100
// }


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
                // PWM_MOTOR = OFF;
            }
        }
        else
        {
            PORTA_SH.IND_LED = ON; // within On part of duty cycle
            // PWM_MOTOR = ON;
        }
        if (countdown)
        {
            countdown--;
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


    // PORTA_SH.DAT_LED = ON;
    // PORTA_SH.CLK_LED = ON;
    PORTA_SH.byte = 0;

    //setup PWM
    PWM_MOTOR_PIN = OUTPUT;
    PWM_MOTOR = OFF;
    TIMER2_CONTROL = (ON<<TIMER2_ON) | (PRESCALE_1<<TIMER_CLOCK_PRESCALE); // Timer 2 register
    PWM_PERIOD = 199; // 5kHz
    pwm_duty_cycle = calcPWM(PWM_PERIOD, 0);
    PWM_CONTROL = (SINGLE_OUTPUT<<PWM_MODE) | (ACTIVE_HIGH_ACTIVE_HIGH<<PWM_OUTPUT) | ((pwm_duty_cycle & 0b11)<<PWM_DUTYCYCLE_LSB); //PWM register set
    PWM_DUTYCYCLE_MSB = (unsigned char)(pwm_duty_cycle>>2);
    

    // Set up timer0
    // calculate intial for accurate timing $ inital = TimerMax-((Delay*Fosc)/(Prescaler*4))
    TIMER0_COUNTER = TIMER0_INITIAL; // set counter
    TIMER0_CLOCK_SCOURCE = INTERNAL; // internal clock
    PRESCALER = 0; // enable prescaler for Timer0
    PS2=0; PS1=1; PS0=0; // Set prescaler to 1:8
    TIMER0_INTERRUPT = ON; // enable timer0 interrupts

    //Set up ADC
    MOTOR_READING_PIN = INPUT;
    MOTOR_READING_TYPE = ANALOG;
    ADC_VOLTAGE_REFERENCE = INTERNAL;
    ADC_CHANNEL2 = 1; ADC_CHANNEL1 = 0; ADC_CHANNEL0 = 1; // Set the channel to AN3 (where the POT is)
    ADC_CLOCK_SOURCE2 = 0; ADC_CLOCK_SOURCE1 = 0; ADC_CLOCK_SOURCE0 = 1; // Set the clock rate of the ADC
    ADC_OUTPUT_FORMAT = RIGHT; // right Shifted ADC_RESULT_HIGH contains the first 2 bits
    ADC_INTERRUPT = OFF; // by default these aren't necessary
    ADC_ON = ON; // turn it on

    //setup flashing led
    led_duty_cycle = 185; //[ms]

    //turn on interrupts
    GLOBAL_INTERRUPTS = ON;

    PORTA_SH.CLK_LED = ON;

    while (1)
    {
        if (ECCPASE)
        {
            PORTA_SH.DAT_LED = ON;
        }

        if (!countdown) // time to take a measurement of the motor speed/voltage
        {
            GLOBAL_INTERRUPTS = OFF;
            countdown = COUNTDOWN_TIME;
            PWM_CONTROL = OFF; // turn off the PWM
            TIMER2_INTERRUPT_FLAG = 0;
            TIMER2 = 0; // Set timer to 0
            // CLK_LED = OFF; //TTT

            while(!TIMER2_INTERRUPT_FLAG); // wait until timer2 equals PWM_PERIOD, anbout 200us
            // CLK_LED = ON; //TTT
            TIMER2_INTERRUPT_FLAG = 0;

            GO_DONE = 1; //begin an ADC reading
            while(GO_DONE); // wait until the measurement is made

            sample.reading1_array[1] = ADC_RESULT_HIGH;
            sample.reading1_array[0] = ADC_RESULT_LOW;

            if (((unsigned long int)sample.reading1*100)/max_voltage < speed)
            {
                if (pwm_duty_cycle>(0x3FF-10)) // if we can't safely add
                {
                    pwm_duty_cycle = 0x3FF; //be max
                }
                else
                {
                    pwm_duty_cycle += 10;
                }
                
            }
            else
            {
                pwm_duty_cycle -= 10;
            }
            PWM_DUTYCYCLE_MSB = (unsigned char)(pwm_duty_cycle>>2);


            //reset
            PWM_CONTROL = (SINGLE_OUTPUT<<PWM_MODE) | (ACTIVE_HIGH_ACTIVE_HIGH<<PWM_OUTPUT) | ((pwm_duty_cycle & 0b11)<<PWM_DUTYCYCLE_LSB); //PWM register set
            TIMER2 = 0;
            GLOBAL_INTERRUPTS = ON;

        }

        PORTA = PORTA_SH.byte; //write out IO register to avoid read-modify-write errors

    }

    return;
}
