This is code for the olimex board E407 with Ethernet and a STM32F407ZGT6

https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware

I will try to add a working FreeRTOS environment to it, so we can re-use this project.
Current status :
Setup board to 120MHz or 168MHz
Runs with latest CMSIS and SPL from ST.
FreeRTOS runs stable.
ARM math works, including DSP functions

- todo check if I2C works
- make tests

Features of the board :
STM32F407ZGT6 Cortex-M4 210DMIPS,
1MB Flash,
196KB RAM,
3 x 12-bit 2.4 MSPS A/D,
2 x 12-bit D/A converters,
USB OTG FS and USB OTG HS,
14 timers,
3 x SPI,
3 x I2C,
Ethernet,
2 CANs,
114 GPIOs,
Camera interface

Price about 39 Euro.

/* Note to SELF :
 * choose the correct target in stm32f4xx.h eg STM32F40_41xxx for the olimex stm32f407 ethernet board
 * choose the correct crystal frequency 12MHz #define HSE_VALUE ((uint32_t)12000000)  Value of the External oscillator in Hz
 * double check the PLL values in system_stm32f4xx.c
 * example for olimex 12MHz external
 * #define PLL_M      12
 * #define PLL_N      336
 * #define PLL_P      2
 * USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ
 * #define PLL_Q      7
*/
