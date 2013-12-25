#include <p32xxxx.h>
#include <plib.h>
#include "U1.h"

/*
 * UART1 init function.
 * Works well at baud 9600 & 500k with pb_clock of 50M
 * Must configure PPS for U1RX and U1TX after calling init
 */
float U1_init(unsigned int pb_clock, int baud)
{
    float actual_baud;

    U1MODE = BIT_15 | BIT_3; // UART1 EN | 4x multiplier
    U1STA |= BIT_12 | BIT_10; // EN RX and TX
    U1BRG = (pb_clock / (4 * baud)) - 1;
    actual_baud = (float)pb_clock / (4 * (U1BRG + 1));

    return actual_baud;
}

/* U1_write() transmits a string to the UART2 TX pin MSB first
 *
 * Inputs: *buffer = string to transmit */
int U1_write(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while( size)
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer

        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }

    while( !U1STAbits.TRMT);        // wait for last transmission to finish

    return 0;
}

/* U1_read() is a blocking function that waits for data on
 *  the UART1 RX buffer and then stores all incoming data into *buffer
 *
 * Note that when a carriage return '\r' is received, a nul character
 *  is appended signifying the strings end
 *
 * Inputs:  *buffer = Character array/pointer to store received data into
 *          max_size = number of bytes allocated to this pointer
 * Outputs: Number of characters received */
unsigned int U1_read(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;

    /* Wait for and store incoming data until either a carriage return is received
     *   or the number of received characters (num_chars) exceeds max_size */
    while(num_char < max_size)
    {
        while( !U1STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer

        // insert nul character to indicate end of string
        if( *buffer == '\r'){
            *buffer = '\0';
            break;
        }

        buffer++;
        num_char++;
    }

    return num_char;
} // END SerialReceive()
