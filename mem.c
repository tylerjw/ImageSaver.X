#include <plib.h>
#include "mem.h"


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
    mPORTFSetBits(WE1 | WE2 | CE1 | CE2 | CLK);
    mPORTDWrite(data); // write data
    //__asm("nop");
    mPORTFClearBits(WE1 | WE2 | CE1 | CE2 | CLK);
    //__asm("nop");
}

void inline __attribute__((always_inline)) mem_write_640_1(unsigned int *data)
{
    int i;
    mPORTDDirection(0xff00); // 0 - output, 1 - input
    mPORTDWrite(0x0000);
    mPORTFSetBits(WE2 | CE2);
    for(i = 0; i < 640; i++)
    {
        mPORTFSetBits(WE1 | CE1 | CLK);
        mPORTDWrite(*(data++));
        mPORTFClearBits(WE1 | CE1 | CLK);
        __asm("nop");
    }
}

void inline __attribute__((always_inline)) mem_write_640_2(unsigned int *data)
{
    int i;
    mPORTDDirection(0x00ff); // 0 - output, 1 - input
    mPORTDWrite(0x0000);
    mPORTFSetBits(WE1 | CE1);
    for(i = 0; i < 640; i++)
    {
        mPORTFSetBits(WE2 | CE2 | CLK);
        mPORTDWrite(*(data++));
        mPORTFClearBits(WE2 | CE2 | CLK);
        __asm("nop");
    }
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
    mPORTFClearBits(CLK);
    __asm("nop");
    __asm("nop");
    __asm("nop");
    __asm("nop");

    return mPORTDRead();
}