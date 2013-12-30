/* 
 * File:   timer1.h
 * Author: shiny
 *
 * Created on December 27, 2013, 11:59 AM
 */

#ifndef TIMER1_H
#define	TIMER1_H

#ifdef	__cplusplus
extern "C" {
#endif

void inline __attribute__((always_inline)) timer1_init();
void inline __attribute__((always_inline)) timer1_start_us();
float inline __attribute__((always_inline)) timer1_end_us();
void inline __attribute__((always_inline)) timer1_delay_ms(unsigned int ms);
void inline __attribute__((always_inline)) timer1_delay_us(unsigned int us);

#ifdef	__cplusplus
}
#endif

#endif	/* TIMER1_H */

