#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>

extern FILE uart_output;
extern FILE uart_input;
int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);

void uart_init();



#endif
