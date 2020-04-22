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

#define LED_PERIOD 100
unsigned short int led_duty_cycle = 25; // Duty cycle of LED as on_time[ms]
volatile unsigned short int led_duty_cycle_counter = 0;

unsigned short int pwm_duty_cycle = 0;//0x3FF; //[ms]
// unsigned short int pwm_period = 500;
volatile unsigned short int speed = 375; // 0->100
unsigned short int in_voltage = 1001; // 16V, the highest this should receive, thus the default
unsigned short int max_voltage = 501; // 7.6V max voltage the motor is allowed to run at (95% of 8V)
unsigned short int min_voltage = 147; // 2.4V min voltage
#define COUNTDOWN_TIME (unsigned char)5 // [ms] represents the amount of time to check the voltage of the motor. 
volatile unsigned char countdown = COUNTDOWN_TIME;
volatile __bit measure_motor;
volatile __bit measure_supply;

volatile __bit proportion_set_mode; // indicates whether we are setting the proportion constant. This involves moving the speed up and down and reading the constant off of the pot. The error will be periodically reset
#define SPEED_CHANGE_RATE (unsigned short int)500 //[ms]
volatile unsigned short int speed_change_count = SPEED_CHANGE_RATE;
signed short int speed_delta = 30; //[Vadc]
#define CLEAR_ERROR_RATE 10
unsigned char clear_error_count = CLEAR_ERROR_RATE;

//control variables
signed long int error = 0;
unsigned char proportion_constant = 156;

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

union reading // a union allowing byte combination from the adc
{
    // arranged this way for memory locations to match
    unsigned short int readings[3];
    struct
    {
        unsigned char reading3_array[2]; // third adc result
        unsigned char reading2_array[2]; // second adc result
        unsigned char reading1_array[2]; // first adc result.
    };
    struct
    {
        unsigned short int reading3; //reading3_array[1] # reading3_array[0]
        unsigned short int reading2; //reading2_array[1] # reading2_array[0]
        unsigned short int reading1; //reading1_array[1] # reading1_array[0]
    };
} sample;
// Save memory!

enum CALC_PWM_PARAMS {ACTIVE_LOW=0, ACTIVE_HIGH=1};
unsigned short int calcPWM(unsigned char period, unsigned short int ratio, unsigned char active_high)
{
    unsigned char max_ratio = 95;
    unsigned char min_ratio = 1;

    if (ratio >= max_ratio)
    {
        ratio = max_ratio;
    }
    else if (ratio <= min_ratio)
    {
        ratio = min_ratio;
    }

    if (active_high)
    {
        return (4*((__uint24)period+1)*ratio)/100;
    }
    else // active low
    {
        return (4*((__uint24)period+1)*(100-ratio))/100;
    }
}

unsigned short int medianValue(unsigned short int samples[]) // returns the index of the middle value
{
   if (samples[0] > samples[1]) 
   {
        if (samples[1] > samples[2])
        {
            return samples[1];
        } 
        else if (samples[0] > samples[2])
        {
            return samples[2];
        } 
        else 
        {
            return samples[0];
        }
    } 
    else 
    {
        if (samples[0] > samples[2]) 
        {
            return samples[0];
        } 
        else if (samples[1] > samples[2]) 
        {
            return samples[2];
        } 
        else 
        {
            return samples[1];
        }
    }
}

void __interrupt() ISR()
{
    if (TIMER1_INTERRUPT_FLAG)
    {
        if (!measure_supply)
        {
            CLK_LED = 1;
            GO_DONE = 1;
            measure_motor = 1;
        }
       
        TIMER1_INTERRUPT_FLAG = 0;
        TIMER1_INTERRUPT = OFF;
        
    }

    if (TIMER2_INTERRUPT_FLAG)
    {
        if (!measure_supply)
        {
            CLK_LED = 0;
            TIMER1_INTERRUPT_FLAG = 0;
            TIMER1_H = 0xFF; TIMER1_L = 0xDD;
            TIMER1_INTERRUPT = ON;
        }
        
        TIMER2_INTERRUPT_FLAG = 0;
    }

    if (TIMER0_INTERRUPT_FLAG) // if the timer0 interrupt flag was set (timer0 triggered)
    {
        TIMER0_INTERRUPT_FLAG = CLEAR; // clear interrupt flag since we are dealing with it
        TIMER0_COUNTER = TIMER0_INITIAL + 2; // reset counter, but also add 2 since it takes 2 clock cycles to get going
        // move counters, which is the job of this timer interrupt
        led_duty_cycle_counter++; // increment the led counter
        // PWM_MOTOR = ~PWM_MOTOR;
        
        if (led_duty_cycle_counter >= led_duty_cycle)
        {
            if (led_duty_cycle_counter >= LED_PERIOD)
            {
                led_duty_cycle_counter -= LED_PERIOD; //reset led counter safely
                // led_state = ON; // we are in the ON part of the duty cycle
                if (!measure_motor)
                {
                    measure_supply = 1;
                }
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
        if (proportion_set_mode)
        {
            if (speed_change_count)
            {
                speed_change_count--;
            }
            else
            {
                speed_change_count = SPEED_CHANGE_RATE;
                speed +=speed_delta;
                if (speed > max_voltage)
                {
                    speed_delta = -1*speed_delta;
                    speed += 2*speed_delta;
                    clear_error_count--;
                }
                else if (speed < min_voltage)
                {
                    speed_delta = -1*speed_delta;
                    speed += 2*speed_delta;
                }
            }
        }

        
        // IND_LED = IND_led_state;
        // LED = led_test_state; //TTT
        // IND_LED = ON;
    }

}

void main() {

    signed long int ratio;
    proportion_set_mode = 0;

    //set up IO
    DAT_LED_TYPE = DIGITAL;
    DAT_LED_PIN = OUTPUT;

    CLK_LED_TYPE = DIGITAL;
    CLK_LED_PIN = OUTPUT;

    IND_LED_TYPE = DIGITAL;
    IND_LED_PIN = OUTPUT;


    PORTA_SH.byte = 0;
    // PORTA_SH.DAT_LED = ON;
    PORTA_SH.CLK_LED = ON;

    //setup PWM and timer 2
    PWM_MOTOR_PIN = OUTPUT;
    PWM_MOTOR = OFF;
    TIMER2_CONTROL = (ON<<TIMER2_ON) | (PRESCALE_16<<TIMER_CLOCK_PRESCALE); // Timer 2 register
    PWM_PERIOD = 222; // 280Hz
    pwm_duty_cycle = calcPWM(PWM_PERIOD, 100, ACTIVE_LOW);
    PWM_CONTROL = (SINGLE_OUTPUT<<PWM_MODE) | (ACTIVE_LOW_ACTIVE_LOW<<PWM_OUTPUT) | ((pwm_duty_cycle & 0b11)<<PWM_DUTYCYCLE_LSB); //PWM register set
    PWM_DUTYCYCLE_MSB = (unsigned char)(pwm_duty_cycle>>2);
    PERIPHAL_INTERRUPT = ON;
    TIMER2_INTERRUPT = ON;

    //timer1
    TIMER1_H =0x00; TIMER1_L = 0x00;
    TIMER1_INTERRUPT = OFF;
    TIMER1 = ON;

    

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
    ADC_CHANNEL2 = 1; ADC_CHANNEL1 = 0; ADC_CHANNEL0 = 1; // Set the channel to AN5 (where the motor feedback is)
    ADC_CLOCK_SOURCE2 = 0; ADC_CLOCK_SOURCE1 = 0; ADC_CLOCK_SOURCE0 = 1; // Set the clock rate of the ADC
    ADC_OUTPUT_FORMAT = RIGHT; // right Shifted ADC_RESULT_HIGH contains the first 2 bits
    ADC_INTERRUPT = OFF; // by default these aren't necessary
    ADC_ON = ON; // turn it on

    //setup flashing led
    // led_duty_cycle = 185; //[ms]

    //turn on interrupts
    GLOBAL_INTERRUPTS = ON;

    // PORTA_SH.CLK_LED = ON;

    while (1)
    {
        // if (0 && !countdown) // time to take a measurement of the motor speed/voltage
        // {
        //     GLOBAL_INTERRUPTS = OFF;
        //     countdown = COUNTDOWN_TIME;
        //     PWM_CONTROL = OFF; // turn off the PWM
        //     TIMER2_INTERRUPT_FLAG = 0;
        //     TIMER2 = 0; // Set timer to 0
        //     // CLK_LED = OFF; //TTT


        //     while(!TIMER2_INTERRUPT_FLAG); // wait until timer2 equals PWM_PERIOD, anbout 200us
        //     // CLK_LED = ON; //TTT
            
        //     //reset
        //     // TIMER2 = 0;
        //     PWM_CONTROL = (SINGLE_OUTPUT<<PWM_MODE) | (ACTIVE_HIGH_ACTIVE_HIGH<<PWM_OUTPUT) | ((pwm_duty_cycle & 0b11)<<PWM_DUTYCYCLE_LSB); //PWM register set
        //     TIMER2_INTERRUPT_FLAG = 0;

        //     GO_DONE = 1; //begin an ADC reading
        //     CLK_LED = OFF;
        //     while(GO_DONE); // wait until the measurement is made

        //     GO_DONE = 1; //begin an ADC reading
        //     CLK_LED = ON;
        //     // save the previous sample
        //     sample.reading1_array[1] = ADC_RESULT_HIGH;
        //     sample.reading1_array[0] = ADC_RESULT_LOW;
        //     while(GO_DONE); // wait until the measurement is made


        //     GO_DONE = 1; //begin an ADC reading
        //     CLK_LED = OFF;
        //     // save the previous sample
        //     sample.reading2_array[1] = ADC_RESULT_HIGH;
        //     sample.reading2_array[0] = ADC_RESULT_LOW;
        //     while(GO_DONE); // wait until the measurement is made
        //     CLK_LED = ON;


        //     //save final sample
        //     sample.reading3_array[1] = ADC_RESULT_HIGH;
        //     sample.reading3_array[0] = ADC_RESULT_LOW;

        //     // pwm_duty_cycle = calcPWM(PWM_PERIOD, speed); // no control, speed is a ratio

        //     // if (((unsigned long int)sample.reading1*100)/max_voltage < speed)
        //     // {
        //     //     if (pwm_duty_cycle>(0x3FF-10)) // if we can't safely add
        //     //     {
        //     //         pwm_duty_cycle = 0x3FF; //be max
        //     //     }
        //     //     else
        //     //     {
        //     //         pwm_duty_cycle += 10;
        //     //     }
                
        //     // }
        //     // else
        //     // {
        //     //     pwm_duty_cycle -= 10;
        //     // }
        //     // PWM_DUTYCYCLE_MSB = (unsigned char)(pwm_duty_cycle>>2);


        //     // error = speed - ((unsigned long int)medianValue(sample.readings)*100)/max_voltage;
        //     // pwm_duty_cycle = calcPWM(PWM_PERIOD, speed+(((signed long int)proportion_constant*error)/100));

        //     error = (signed long int)speed - medianValue(sample.readings);
        //     // if (!clear_error_count)
        //     // {
        //     //     error = 0;
        //     // }
        //     ratio = ((speed+(error*proportion_constant)/100)*(unsigned long int)100)/max_voltage;
        //     // ratio = ((unsigned long int)speed*100)/in_voltage;

        //     // if (!proportion_set_mode && (ratio > 100 || ratio < 0))
        //     // {
        //     //     ratio = ((unsigned long int)speed*100)/in_voltage;
        //     // }
            
        //     pwm_duty_cycle = calcPWM(PWM_PERIOD, ratio);
        //     PWM_DUTYCYCLE_MSB = (unsigned char)(pwm_duty_cycle>>2);
        //     // reset this so the lsb is in the mix
        //     PWM_CONTROL = (SINGLE_OUTPUT<<PWM_MODE) | (ACTIVE_LOW_ACTIVE_LOW<<PWM_OUTPUT) | ((pwm_duty_cycle & 0b11)<<PWM_DUTYCYCLE_LSB);

        //     //reset
        //     // PWM_CONTROL = (SINGLE_OUTPUT<<PWM_MODE) | (ACTIVE_HIGH_ACTIVE_HIGH<<PWM_OUTPUT) | ((pwm_duty_cycle & 0b11)<<PWM_DUTYCYCLE_LSB); //PWM register set
            
        //     GLOBAL_INTERRUPTS = ON;

        // }

        if (measure_motor)
        {
            // CLK_LED = OFF;
            PORTA_SH.DAT_LED = ~PORTA_SH.DAT_LED;
            
            GLOBAL_INTERRUPTS = OFF;

            // GO_DONE = 1; //begin an ADC reading
            // CLK_LED = OFF;
            while(GO_DONE); // wait until the first measurement (initiated by the timer) is made
            
            GO_DONE = 1; //BEGIN second measurement

            sample.reading1_array[1] = ADC_RESULT_HIGH; //SAVE first measurement
            sample.reading1_array[0] = ADC_RESULT_LOW;

            while(GO_DONE);

            GO_DONE = 1; //BEGIN third measurement

            sample.reading2_array[1] = ADC_RESULT_HIGH; //SAVE second measurement
            sample.reading2_array[0] = ADC_RESULT_LOW;

            while(GO_DONE);

            // CLK_LED = ON;

            sample.reading3_array[1] = ADC_RESULT_HIGH; //SAVE third measuement
            sample.reading3_array[0] = ADC_RESULT_LOW;

            GLOBAL_INTERRUPTS = ON;
            measure_motor = 0;

            
            error = (signed long int)speed - medianValue(sample.readings);
            error = (error*proportion_constant)/100;

            ratio = speed+error;

            if (ratio <= 0) // error magnitude too large
            {
                ratio = speed;
            }
            // else if (ratio > in_voltage)
            // {
            //     ratio = 100;
            // }
            else
            {
                ratio = ((ratio)*22)/in_voltage;
            }

            
            pwm_duty_cycle = calcPWM(PWM_PERIOD, ratio, ACTIVE_LOW);
            PWM_DUTYCYCLE_MSB = (unsigned char)(pwm_duty_cycle>>2);
            // reset this so the lsb is in the mix
            PWM_CONTROL = (SINGLE_OUTPUT<<PWM_MODE) | (ACTIVE_LOW_ACTIVE_LOW<<PWM_OUTPUT) | ((pwm_duty_cycle & 0b11)<<PWM_DUTYCYCLE_LSB);



            // // CLK_LED = ON;
            // GLOBAL_INTERRUPTS = ON;
        }

        if (measure_supply)
        {
            
            if (PWM_MOTOR) // if supply is high
            {
                

                GLOBAL_INTERRUPTS = OFF;
                GO_DONE = 1;
                while (GO_DONE);
                CLK_LED = 1;
                sample.reading1_array[1] = ADC_RESULT_HIGH;
                sample.reading1_array[0] = ADC_RESULT_LOW;

                if (sample.reading1 > 0)
                {
                    
                    // in_voltage = sample.reading1;
                    in_voltage = sample.reading1;
                    
                }

                GLOBAL_INTERRUPTS = ON;

                measure_motor = 0;
                measure_supply = 0;

            }
            
        }

        //read pot for P gain

        proportion_set_mode = 0;

        ADC_CHANNEL2 = 1; ADC_CHANNEL1 = 0; ADC_CHANNEL0 = 0; // Set the channel to AN4 (where the POT is)
        ADC_PAUSE; // wait for small amount of time for the channels to redirect
        GO_DONE = 1; // begin an ADC read
        while (GO_DONE); // wait for the adc to finish
        sample.reading1_array[1] = ADC_RESULT_HIGH;
        sample.reading1_array[0] = ADC_RESULT_LOW;
        // proportion_constant = sample.reading1 >> 2; // 8 bits is enough
        // speed = ((long int)sample.reading1*97)/1000;     //((unsigned long int)sample.reading1*312)/100 + 147; // convert speed to desired range
        // speed = 1023;

        ADC_CHANNEL2 = 1; ADC_CHANNEL1 = 0; ADC_CHANNEL0 = 1; // Set the channel back to AN5 (where the Motor feedback is)
        ADC_PAUSE;

        if (proportion_set_mode)
        {
            proportion_constant = (unsigned char)(((unsigned long int)sample.reading1*195)/1000);
        }
        else
        {
            proportion_constant = 28;
            speed = (unsigned short int)(((unsigned long int)sample.reading1 * max_voltage)/1024);
        }
        
        

        PORTA = PORTA_SH.byte; //write out IO register to avoid read-modify-write errors

    }

    return;
}
