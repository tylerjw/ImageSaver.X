/* 
 * File:   mem.h
 * Author: shiny
 *
 * Created on January 3, 2014, 9:07 AM
 */

#ifndef MEM_H
#define	MEM_H

#ifdef	__cplusplus
extern "C" {
#endif

#define CCLR    BIT_2
#define CLK     BIT_3
#define CE1     BIT_0
#define CE2     BIT_1
#define WE1     BIT_8
#define WE2     BIT_7

void mem_init();
void inline __attribute__((always_inline)) mem_reset_addr();
void inline __attribute__((always_inline)) mem_write_init();
void inline __attribute__((always_inline)) mem_write_16(unsigned int data);
void inline __attribute__((always_inline)) mem_write_640_1(unsigned int *data); // writes 640 8bit values to bank 1
void inline __attribute__((always_inline)) mem_write_640_2(unsigned int *data); // writes 640 8bit values to bank 2
//void inline __attribute__((always_inline)) mem_read_640_1(unsigned int *data); // reads 640 8bit values from bank 1
void inline __attribute__((always_inline)) mem_read_init();
unsigned int inline __attribute__((always_inline)) mem_read_16();


#ifdef	__cplusplus
}
#endif

#endif	/* MEM_H */

