/* 
 * File:   U1.h
 * Author: shiny
 *
 * Created on December 24, 2013, 3:54 PM
 */

#ifndef U1_H
#define	U1_H

#ifdef	__cplusplus
extern "C" {
#endif

float U1_init(unsigned int pb_clock, int baud);
unsigned int U1_read(char *buffer, unsigned int max_size);
int U1_write(const char *buffer);

#ifdef	__cplusplus
}
#endif

#endif	/* U1_H */

