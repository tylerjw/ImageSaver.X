/*
 * File:   main.c
 * Author: tylerjw
 *
 * Created on December 19, 2013, 11:59 AM
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright � 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/


#include <p32xxxx.h>
#include <plib.h>
#include <stdio.h>
#include <stdbool.h>
#include "timer1.h"
#include "U1.h"
#include "mem.h"

#define SYS_CLK 100000000L // 100MHz

float time_base = 0.0;

//	Function Prototypes
int main(void);
void delay(volatile unsigned int count);
void mem_test_16();
void mem_test_lines();

int main(void) {
    char buffer[80];
    unsigned int pb_clock;
    float actual_baud;
    const int baud = 500000; // max baud rate using arduino interface 

    pb_clock = SYSTEMConfigPerformance(SYS_CLK); // if sys_clock > 100MHz, pb_clock = sys_clock/2 else pb_clock = sys_clock

    PORTSetPinsDigitalOut(IOPORT_E, BIT_4); // led

    // setup UART
    PPSUnLock;
    PPSInput(1,U1RX,RPF4); // Rx - F4 (pin 49) 5V tolerent
    PPSOutput(2,RPF5,U1TX); // Tx - F5 (pin 50) 5V tolerent
    PPSLock;
    
    actual_baud = U1_init(pb_clock, baud);
    /*
    sprintf(buffer, "SYSCLK: %d\r\n", SYS_CLK);
    U1_write(buffer);
    sprintf(buffer, "PBCLK: %d\r\n", pb_clock);
    U1_write(buffer);
    sprintf(buffer, "U1BRG: %d\r\n", U1BRG);
    U1_write(buffer);
    sprintf(buffer, "target baud: %d\r\n", baud);
    U1_write(buffer);
    sprintf(buffer, "actual baud: %f\r\n", actual_baud);
    U1_write(buffer);
    */
    U1_write("Initializing timer1... \r\n");
    timer1_init();
    U1_write("Running memory test... \r\n");
    mem_test_lines();

    while (1) {
        mPORTEWrite(0);
        timer1_delay_ms(1000);
        mPORTEWrite(BIT_4);
        timer1_delay_ms(1000);
    }
}

void delay(volatile unsigned int count)
{
    while(--count);
}

#define values 307200

void mem_test_16()
{
    //const unsigned int values = 2000;
//    unsigned int input_data[values];
//    unsigned int test_data[values];
    unsigned int test_data;
    unsigned int input_data = 0x0000;
    char buffer[80];
    unsigned int i;
    float time;
    bool passed = true;

    // load test data
//    for(i=0;i<values;i++)
//        input_data[i] = 0xffff - i;

    timer1_start_us();
    time_base = timer1_end_us();
//    sprintf(buffer, "time_base: %f us\r\n", time_base);
//    U1_write(buffer);
    mem_init();
    mem_write_init();
    U1_write("Writing 307200 16-bit values...\r\n");
    timer1_start_us();
    for(i=0;i<values;i++)
    {
        //input_data = (unsigned int)(0xffff - i) & 0xffff;
        mem_write_16(input_data);
    }
    time = timer1_end_us();
    sprintf(buffer, "Total Write: %f us, Average: %f us, Freq: %f MHz\r\n", time, time / values, 1.0/(time/values));
    U1_write(buffer);
//    mPORTEWrite(BIT_4);
    mem_reset_addr();
    mem_read_init();
    U1_write("Reading 307200 16-bit values...\r\n");
//    timer1_start_us();
    for(i=0;i<values;i++)
    {
        test_data = mem_read_16();
        //input_data = (unsigned int)(0xffff - i) & 0xffff;
        if(input_data == test_data)
        {
            //sprintf(buffer,"%d - Wrote: 0x%04x - Read: 0x%04x - Pass!\r\n", i, (0xffff - i), test_data);
            //U1_write(buffer);
        }
        else
        {
            passed = false;
            sprintf(buffer,"%d - Wrote: 0x%04x - Read: 0x%04x - Fail!\r\n", i, input_data, test_data);
            U1_write(buffer);
        }
    }
//    time = timer1_end_us();
//    sprintf(buffer, "Total Read: %f us, Average: %f us, Freq: %f MHz\r\n", time, time / values, 1.0/(time/values));
//    U1_write(buffer);
//    mPORTEWrite(0);
//    U1_write("Comparing all read values to those written...\r\n");
//    for(i=0; i < values; i++)
//    {
//        if(input_data[i] == test_data[i])
//        {
//            sprintf(buffer,"%d - Wrote: 0x%04x - Read: 0x%04x - Pass!\r\n", i, input_data[i], test_data[i]);
//            //U1_write(buffer);
//        }
//        else
//        {
//            passed = false;
//            sprintf(buffer,"%d - Wrote: 0x%04x - Read: 0x%04x - Fail!\r\n", i, input_data[i], test_data[i]);
//            U1_write(buffer);
//        }
//    }
    if(passed)
    {
        U1_write("Passed all tests!\r\n");
    }
}

void mem_test_lines()
{
    unsigned int output_data[640];
    unsigned int input_data[640];
    int i;
    bool passed = true;
    char buffer[80];
    float time;

    for(i=0;i<640;i++)
        output_data[i] = (i) & 0xff | ((i << 8) & 0xff00);

    sprintf(buffer, "output data 2: 0x%04x\r\n", output_data[2]);
    U1_write(buffer);

    timer1_start_us();
    time_base = timer1_end_us();

    mem_init();

    //mem_reset_addr();
    timer1_start_us();
    mem_write_640_2(output_data);
    time = timer1_end_us();
    sprintf(buffer, "Total Write 2: %f us, Average: %f us, Freq: %f MHz\r\n", time, time / 640, 1.0/(time/640));
    U1_write(buffer);
    
    mem_reset_addr();
    timer1_start_us();
    mem_write_640_1(output_data);
    time = timer1_end_us();
    sprintf(buffer, "Total Write 1: %f us, Average: %f us, Freq: %f MHz\r\n", time, time / 640, 1.0/(time/640));
    U1_write(buffer);

    mem_reset_addr();
    mem_read_init();
    for(i=0;i<640;i++)
        input_data[i] = mem_read_16();

    for(i=0;i<640;i++)
    {
        if(output_data[i] != input_data[i]) // bad data
        {
            passed = false;
            sprintf(buffer,"%d - Wrote: 0x%04x - Read: 0x%04x - Fail!\r\n", i, output_data[i], input_data[i]);
            U1_write(buffer);
        }
    }
    if(passed)
    {
        U1_write("Passed All Tests\r\n");
    }
}
