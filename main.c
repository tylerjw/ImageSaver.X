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
#include "U1.h"

#define SYS_CLK 100000000L // 100MHz

#define CCLR    BIT_2
#define CLK     BIT_3
#define CE1     BIT_0
#define CE2     BIT_1
#define WE1     BIT_8
#define WE2     BIT_7

//	Function Prototypes
int main(void);
void delay(volatile unsigned int count);
void mem_init();
void inline __attribute__((always_inline)) mem_reset_addr();
void inline __attribute__((always_inline)) mem_write_init();
void inline __attribute__((always_inline)) mem_write_16(unsigned int data);
void inline __attribute__((always_inline)) mem_read_init();
unsigned int inline __attribute__((always_inline)) mem_read_16();
void mem_test_16();

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

    mem_test_16();

    while (1) {
        mPORTEWrite(0);
        U1_write("Hello World!\r\n");
        delay(SYS_CLK/4);
        mPORTEWrite(BIT_4);
        delay(SYS_CLK/4);
    }
}

void delay(volatile unsigned int count)
{
    while(--count);
}

void mem_init()
{
    PORTSetPinsDigitalOut(IOPORT_F, CCLR | CLK | WE1 | WE2 | CE1 | CE2);
    mPORTFWrite(0x0000);
    mem_reset_addr();
}

void inline __attribute__((always_inline)) mem_reset_addr()
{
    mPORTFClearBits(CLK | CCLR);
    mPORTFSetBits(CCLR);
}

void inline __attribute__((always_inline)) mem_write_init()
{
    mPORTDSetPinsDigitalOut(0xFFFF);
    mPORTDWrite(0x0000);
}

void inline __attribute__((always_inline)) mem_write_16(unsigned int data)
{
    mPORTFSetBits(WE1 | WE2 | CE1 | CE2);
    mPORTFSetBits(CLK); // increment address
    mPORTDWrite(data); // write data
    __asm("nop");
    mPORTFClearBits(WE1 | WE2 | CE1 | CE2);
    mPORTFClearBits(CLK);
}

void inline __attribute__((always_inline)) mem_read_init()
{
    mPORTDSetPinsDigitalIn(0xFFFF);
    mPORTFSetBits(WE1 | WE2);
    mPORTFClearBits(CLK | CE1 | CE2);
}

unsigned int inline __attribute__((always_inline)) mem_read_16()
{
    mPORTFSetBits(CLK); // increment address
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");
    return mPORTDRead();
}

void mem_test_16()
{
    unsigned int input_data = 0xFFFF;
    unsigned int test_data;
    char buffer[80];
    mem_init();
    mem_write_init();
    mem_write_16(input_data);
    mPORTEWrite(BIT_4);
    mem_reset_addr();
    mem_read_init();
    test_data = mem_read_16();
    mPORTEWrite(0);
    sprintf(buffer,"Wrote: 0x%04x\r\n", input_data);
    U1_write(buffer);
    sprintf(buffer,"Read back: 0x%04x\r\n", test_data);
    U1_write(buffer);
}
