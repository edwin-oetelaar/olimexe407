#ifndef UTILS_EDWIN_H_INCLUDED
#define UTILS_EDWIN_H_INCLUDED

#include "my_queue.h"
#include <inttypes.h>

#define I2C_TIMEOUT_MAX 10000
#define MEM_DEV_ID_BY_ARDUINO  0x50
#define MEM_DEVICE_WRITE_ADDR (MEM_DEV_ID_BY_ARDUINO<<1)
#define MEM_DEVICE_READ_ADDR  (MEM_DEVICE_WRITE_ADDR + 1)

enum  eepromtype  {M2401,M2402,M2404,M2408,M2416,M2432,M2464,M24128,M24256,M24512};

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
static void HardFault_Handler(void) __attribute__((naked));
void SERIAL_puts(const char *s);
void SERIAL_write(const char *s, uint32_t len);
// static void init_USART6(uint32_t baudrate);
// void USART6_IRQHandler(void);
void init_SERIAL(uint32_t baudrate);

uint8_t Write_24Cxx(uint16_t, uint8_t, uint8_t);
uint8_t Read_24Cxx(uint16_t, uint8_t);

#endif /* UTILS_EDWIN_H_INCLUDED */
