#include <p32xxxx.h>
#include <plib.h>
#include "timer1.h"

volatile int counter = 0;

void inline __attribute__((always_inline)) timer1_init()
{
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2); // configure interrupt
    INTEnableSystemMultiVectoredInt();
}

void inline __attribute__((always_inline)) timer1_start_us()
{

    counter = 0;
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, 50000); //milisecond counter
}

float inline __attribute__((always_inline)) timer1_end_us()
{
    unsigned int tmr = TMR1;
    T1CONCLR = BIT_15; // stop the tomer
    TMR1 = 0x0; // reset counter

    return (counter * 1000) + ((float)tmr / 50.0); // micro seconds
}

void __ISR(4, ipl2) _Timer1Handler(void)
{
    mT1ClearIntFlag();
    counter++;
}

void inline __attribute__((always_inline)) timer1_delay_ms(unsigned int ms)
{
    counter = 0;
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, 50000); //milisecond counter
    while(counter < ms);
    T1CONCLR = BIT_15; // stop the tomer
    TMR1 = 0x0;
}

// 100 us minimum for acuracy
void inline __attribute__((always_inline)) timer1_delay_us(unsigned int us)
{
    counter = (int)(us * 0.022);
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, 50); // micro second interrupts
    while(counter < us);
    T1CONCLR = BIT_15; // stop the tomer
    TMR1 = 0x0;
}