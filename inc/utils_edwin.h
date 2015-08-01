#ifndef UTILS_EDWIN_H_INCLUDED
#define UTILS_EDWIN_H_INCLUDED

#include "my_queue.h"
#include <inttypes.h>
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
static void HardFault_Handler( void ) __attribute__( ( naked ) );
void SERIAL_puts(const char *s);
void SERIAL_write(const char *s, uint32_t len);
static void init_USART6(uint32_t baudrate);
// void USART6_IRQHandler(void);
void init_SERIAL(uint32_t baudrate);
#endif /* UTILS_EDWIN_H_INCLUDED */
