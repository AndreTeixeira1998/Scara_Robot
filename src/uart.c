#include "uart.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 9600UL
#endif

#include <util/setbaud.h>

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
/*FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW); alternativa que nao consegui meter a funcionar*/

void uart_init() {
  // Assign values calculated by setbaud.h
  UBRR0H = UBRRH_VALUE; //isto sao macros
  UBRR0L = UBRRL_VALUE;

#if USE_2X
  UCSR0A |= (1<<U2X0); //se o macro for verdade isto é como os ms da A4988 mete mais rapido
#else
  UCSR0A &= ~((1<<U2X0));
#endif

  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8-bit data, 1 stop bit
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);   // Enable RX and TX

  // escusado de fazer dois inits, por isso meti aqui
  stdout = &uart_output;
  stdin  = &uart_input;
}

int uart_putchar(char c, FILE *stream) {
  if (c == '\n') {
    uart_putchar('\r', stream);
  }
  while (!( UCSR0A & (1<<UDRE0))); // mete no sitio certo para escrever
  UDR0 = c;
  return 0;
}

int uart_getchar(FILE *stream) {
    while (!( UCSR0A & (1<<RXC0))); /* espera até haver data no RX */
    return UDR0;
}
