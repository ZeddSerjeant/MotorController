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

//variables for saving data to the onboard memory
#define PROPORTION_CONSTANT_ADDRESS (unsigned char)0
#define INTEGRAL_CONSTANT_ADDRESS (unsigned char)1

// for timing smaller than a single time loop.
#define TIMER0_INITIAL 118

// represents an LED.
volatile struct LED_TYPE
{
    unsigned short int period;
    unsigned short int duty_cycle;
    unsigned short int counter;
} system_state;


unsigned short int pwm_duty_cycle = 0;//0x3FF; //[ms]
// unsigned short int pwm_period = 500;
volatile unsigned short int speed = 0; // 0->1024, voltage as represented on the POT
unsigned short int in_voltage = 1001; // 16V, the highest this should receive, thus the default
unsigned short int max_voltage = 501; // 7.6V max voltage the motor is allowed to run at (95% of 8V)
unsigned short int min_voltage = 147; // 2.4V min voltage
volatile __bit measure_motor;
volatile __bit measure_supply;
volatile __bit measure_pot;

// when the constants are being set (>1 system mode) then the speed oscillates automatically
//    indicates whether we are setting the proportion constant. This involves moving the speed up and down and reading the 
//    constant off of the pot. The error will be periodically reset
#define SPEED_CHANGE_RATE (unsigned short int)1000 //[ms]
volatile unsigned char system_mode = 0; 
volatile unsigned short int speed_change_count = SPEED_CHANGE_RATE;
signed short int speed_delta = 30; //[Vadc]
volatile __bit clear_errors;
volatile __bit error_state;

//BUTTON
volatile __bit increment_mode; // indicates whether we should move to the next mode
#define BUTTON_BOUNCE (unsigned char)200
volatile unsigned char button_bounce_count = BUTTON_BOUNCE; 

//control variables
signed long int ratio;
//proportion
signed long int error = 0;
unsigned char proportion_constant;
//integral
signed long int error_sum = 0;
unsigned char integral_constant;

// shadow register for PortA, so as to not suffer from read/modify/write errors
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


void EEPROMWrite(unsigned char address, unsigned char data)
{
    while (WR); // wait for a previous write to finish
    EEADR = address; //load the address
    EEDATA = data; // load the data
    GLOBAL_INTERRUPTS = OFF;
    WREN = 1; // enable writes to occur
    EECON2 = 0x55; // each of these bytes is a process required by the hardware
    EECON2 = 0xAA;
    WR = 1; // Initiate the write sequence
    WREN = 0; // disable writes 
    GLOBAL_INTERRUPTS = ON;
}

unsigned char EEPROMRead(unsigned char address)
{
    EEADR = address; // load the address
    RD = 1; // initiate read
    return EEDATA;    
}

//a function (and input defaults) for calculating the the value for the dutycycle register based on the period and a ratio
enum CALC_PWM_PARAMS {ACTIVE_LOW=0, ACTIVE_HIGH=1};
unsigned short int calcPWM(unsigned char period, unsigned short int ratio, unsigned char active_high)
{
    if (active_high)
    {
        return (4*((__uint24)period+1)*ratio)/100;
    }
    else // active low
    {
        return (4*((__uint24)period+1)*(100-ratio))/100;
    }
}

//median of three values
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
    // turned on by timer 2
    if (TIMER1_INTERRUPT_FLAG)
    {
        GO_DONE = 1;
        measure_motor = 1;
       
        TIMER1_INTERRUPT_FLAG = 0;
        TIMER1_INTERRUPT = OFF;
        
    }

    //connected to the PWM
    if (TIMER2_INTERRUPT_FLAG)
    {
        TIMER1_INTERRUPT_FLAG = 0;
        TIMER1_H = 0xFF; TIMER1_L = 0xDD;
        TIMER1_INTERRUPT = ON;
        
        TIMER2_INTERRUPT_FLAG = 0;
    }

    //millisecond interrupt for LED control
    if (TIMER0_INTERRUPT_FLAG) // if the timer0 interrupt flag was set (timer0 triggered)
    {
        TIMER0_INTERRUPT_FLAG = CLEAR; // clear interrupt flag since we are dealing with it
        TIMER0_COUNTER = TIMER0_INITIAL + 2; // reset counter, but also add 2 since it takes 2 clock cycles to get going
        // move counters, which is the job of this timer interrupt
        system_state.counter++; // increment the led counter
        // PWM_MOTOR = ~PWM_MOTOR;
        // measure_pot = 1;
        // measure_supply = 1;
        
        if (system_state.counter >= system_state.period)
        {
            system_state.counter -= system_state.period; //reset led counter safely
            measure_supply = 1; // every so often, ensure the supply is at the right level, and use it to set the ratios more accurately
        }
        if (system_state.counter >= system_state.duty_cycle)
        {
            PORTA_SH.DAT_LED = OFF;
        }
        else
        {
            PORTA_SH.DAT_LED = ON; // within On part of duty cycle
            // PWM_MOTOR = ON;
        }

        //other timings
        if (button_bounce_count)
        {
            button_bounce_count--;
        }
        if (system_mode >= 1)
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
                    clear_errors = 1;
                }
                else if (speed < min_voltage)
                {
                    speed_delta = -1*speed_delta;
                    speed += 2*speed_delta;
                }
            }
        }
    }
    if (BUTTON_INTERRUPT_FLAG)
    {
        BUTTON_INTERRUPT_FLAG = CLEAR; // we are dealing with it
        if (!BUTTON && !button_bounce_count) // button was pressed and therefore this will read low (and we avoided bounce)
        {
            increment_mode = 1;
            button_bounce_count = BUTTON_BOUNCE;
        }
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


    PORTA_SH.byte = 0;

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
    // TIMER1_INTERRUPT = OFF;
    TIMER1 = ON;


    // Set up timer0
    // calculate intial for accurate timing $ inital = TimerMax-((Delay*Fosc)/(Prescaler*4))
    TIMER0_COUNTER = TIMER0_INITIAL; // set counter
    TIMER0_CLOCK_SCOURCE = INTERNAL; // internal clock
    PRESCALER = 0; // enable prescaler for Timer0
    PS2=0; PS1=1; PS0=0; // Set prescaler to 1:8
    TIMER0_INTERRUPT = ON; // enable timer0 interrupts

    //Set up ADC
    // MOTOR_READING_PIN = INPUT;
    // MOTOR_READING_TYPE = ANALOG;
    ADC_VOLTAGE_REFERENCE = INTERNAL;
    ADC_CHANNEL2 = 1; ADC_CHANNEL1 = 0; ADC_CHANNEL0 = 1; // Set the channel to AN5 (where the motor feedback is)
    ADC_CLOCK_SOURCE2 = 0; ADC_CLOCK_SOURCE1 = 0; ADC_CLOCK_SOURCE0 = 1; // Set the clock rate of the ADC
    ADC_OUTPUT_FORMAT = RIGHT; // right Shifted ADC_RESULT_HIGH contains the first 2 bits
    // ADC_INTERRUPT = OFF; // by default these aren't necessary
    ADC_ON = ON; // turn it on

    //setup button
    // BUTTON_PIN = INPUT;
    BUTTON_INTERRUPT = ON;
    RA_INTERRUPT = ON;

    // load system variables
    // proportion_constant = EEPROMRead(PROPORTION_CONSTANT_ADDRESS);
    // integral_constant = EEPROMRead(INTEGRAL_CONSTANT_ADDRESS);
    proportion_constant = 40; // x/100
    integral_constant = 1; // x/100

    //turn on interrupts
    GLOBAL_INTERRUPTS = ON;

    while (1)
    {
        if (measure_motor)
        {            
            GLOBAL_INTERRUPTS = OFF;

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

            // speed = 250;
            
            error = (signed long int)speed - medianValue(sample.readings);
            error_sum += error;
            if (clear_errors)
            {
                clear_errors = 0;
                error = 0;
                error_sum = 0;
                PORTA_SH.CLK_LED = OFF;
            }

            if (error > 200 || error < -200)
            {
                PORTA_SH.IND_LED = ON;
            }
            else
            {
                PORTA_SH.IND_LED = OFF;
            }
            
            if (!error_state)
            {
                ratio = (((signed long int)speed+(error*proportion_constant)/100+(error_sum*integral_constant)/100)*22)/in_voltage;
            }
            else
            {
                ratio = 0;
            }
            
            if (ratio <= 0) // error magnitude too large
            {
                ratio = 1;
            }
            else if (ratio > 100)
            {
                ratio = 95;
            }

            
            pwm_duty_cycle = calcPWM(PWM_PERIOD, ratio, ACTIVE_LOW);
            PWM_DUTYCYCLE_MSB = (unsigned char)(pwm_duty_cycle>>2);
            // reset this so the lsb is in the mix
            PWM_CONTROL = (SINGLE_OUTPUT<<PWM_MODE) | (ACTIVE_LOW_ACTIVE_LOW<<PWM_OUTPUT) | ((pwm_duty_cycle & 0b11)<<PWM_DUTYCYCLE_LSB);


            measure_pot = 1;

        }

        if (measure_supply)
        {
            
            if (PWM_MOTOR) // if supply is high
            {
                

                GLOBAL_INTERRUPTS = OFF;
                GO_DONE = 1;
                while (GO_DONE);
                // CLK_LED = 1;
                sample.reading1_array[1] = ADC_RESULT_HIGH;
                sample.reading1_array[0] = ADC_RESULT_LOW;

                if (sample.reading1 > 0)
                {
                    if (sample.reading1 < max_voltage || sample.reading1 > 1001) // we are less than the highest setting
                    {
                        error_state = 1;
                        PORTA_SH.CLK_LED = ON;
                    }
                    else
                    {
                        error_state = 0;
                        in_voltage = sample.reading1;
                        PORTA_SH.CLK_LED = OFF;
                    }                    
                    
                    
                }

                GLOBAL_INTERRUPTS = ON;

                measure_supply = 0;
                measure_pot = 0;
                measure_motor = 0;
            }
            
        }


        if (measure_pot)
        {
            measure_pot = 0;
            

            GLOBAL_INTERRUPTS = OFF;
            ADC_CHANNEL2 = 1; ADC_CHANNEL1 = 0; ADC_CHANNEL0 = 0; // Set the channel to AN4 (where the POT is)
            ADC_PAUSE; // wait for small amount of time for the channels to redirect
            
            GO_DONE = 1; // begin an ADC read

            while(GO_DONE); // wait until the first measurement (initiated by the timer) is made
            
            GO_DONE = 1; //BEGIN second measurement

            sample.reading1_array[1] = ADC_RESULT_HIGH; //SAVE first measurement
            sample.reading1_array[0] = ADC_RESULT_LOW;

            while(GO_DONE);

            sample.reading2_array[1] = ADC_RESULT_HIGH; //SAVE second measurement
            sample.reading2_array[0] = ADC_RESULT_LOW;

            ADC_CHANNEL2 = 1; ADC_CHANNEL1 = 0; ADC_CHANNEL0 = 1; // Set the channel back to AN5 (where the Motor feedback is)
            
            GLOBAL_INTERRUPTS = ON;

            if (system_mode == 0) // basic speed set mode
            {
                system_state.period = max_voltage;
                system_state.duty_cycle = speed;

                speed = (unsigned short int)((((unsigned long int)sample.reading1 + sample.reading2) * max_voltage)/2048);

                if (increment_mode)
                {
                    increment_mode = 0;

                    if (sample.reading1 < 5)
                    {
                        speed = 150;
                        system_mode++;
                    }
                    else
                    {
                        clear_errors = 1;
                        PORTA_SH.CLK_LED = ON;
                    }
                }

            }
            else if (system_mode == 1) //proportion set mode
            {
                system_state.duty_cycle = 0xFFFF;
                proportion_constant = (unsigned char)(((unsigned long int)sample.reading1*195)/1000);

                if (increment_mode)
                {
                    increment_mode = 0;
                    system_mode = 2;

                    EEPROMWrite(PROPORTION_CONSTANT_ADDRESS, proportion_constant);
                }
            }
            else if (system_mode == 2)
            {
                system_state.duty_cycle = 0;
                integral_constant = (unsigned char)(((unsigned long int)sample.reading1*97)/100);

                if (increment_mode)
                {
                    increment_mode = 0;
                    system_mode = 1;

                    EEPROMWrite(INTEGRAL_CONSTANT_ADDRESS, integral_constant);
                }
            }
        }
                
        // speed = system_state.duty_cycle;
        PORTA = PORTA_SH.byte; //write out IO register to avoid read-modify-write errors

    }

    return;
}
