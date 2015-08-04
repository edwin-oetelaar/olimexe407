/*
 * made by Edwin van den Oetelaar
 * No warranty whatsoever
 * made for personal use, but if you like it, use it
*/

#include <math.h>
#include "arm_math.h"
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "utils_edwin.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"
#include "my_queue.h"
//#include "maple_codec.h"
#include <stdio.h>
#include <inttypes.h>
#include "test_math.h"
// #include "WM_8731.h"

#define USE_DEFAULT_TIMEOUT_CALLBACK 1 /* forces hangup */

const uint32_t baudrate = 921600; // debug console baud rate

void init_LED_GPIO()
{
// enable clocks
// enable pin
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO
    // enable Clocks for APB1 and GPIOB en C
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // de pinnen van de LED instellen
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}


void hardware_LED_Toggle()
{
    static int i = 0;
    static uint32_t x = 0xFFFFFFFF - 10; // check overflow behaviour
    i = !i;
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, i); // RESET
    char buf[40];
    snprintf(buf, sizeof buf, "TimerCallback x=%lu\r\n", x++);
    SERIAL_puts(buf);
}

void vTimerCallback(void *ptr)
{
    /* do something visible */
    hardware_LED_Toggle();
}


int main(void)
{
    init_SERIAL(baudrate); // set up usart6
    SERIAL_puts("USART6 init done\r\n");
    char buf[100];
    /*Scanning...
    I2C device found at address 0x57  EEPROM !
    I2C device found at address 0x68  RTC !
    */
    RCC_ClocksTypeDef RCC_kloks;
    RCC_GetClocksFreq(&RCC_kloks);

    snprintf(buf, sizeof buf, "\r\nCPU clock speeds\r\n"
             "SYSCLK=%"PRIu32"\r\n"
             "HCLK=%"PRIu32"\r\n"
             "PCLK1=%"PRIu32"\r\n"
             "PCLK2=%"PRIu32"\r\n",
             RCC_kloks.SYSCLK_Frequency,
             RCC_kloks.HCLK_Frequency,
             RCC_kloks.PCLK1_Frequency,
             RCC_kloks.PCLK2_Frequency);

    SERIAL_puts(buf);
    SERIAL_puts("GPIO init\r\n");
    init_LED_GPIO();
    SERIAL_puts("GPIO init done\r\n");
    SERIAL_puts("I2C init\r\n");
// we nemen PB9 en PB8 I2C1 omdat die op UEXT zitten
    init_I2C1();
    SERIAL_puts("I2C init done\r\n");
    int i;
    for (i=0; i<10; i++)
    {
        uint8_t x = Read_24Cxx(i,M24512);
        snprintf(buf,sizeof buf,"op pos %d %x\r\n",i,x);
        SERIAL_puts(buf);
    }
    int x = Write_24Cxx(0x01,0xAA,M24512);
    snprintf(buf,sizeof buf,"write op pos 0x01 van i2c geeft %d\r\n",x);
    SERIAL_puts(buf);

    i=0;
    int j=0;
    while ((j= Busy_24Cxx() )!= 0) {

    snprintf(buf,sizeof buf,"still busy %x %d\r\n",j,i++);
    SERIAL_puts(buf);

    }

    x = Read_24Cxx(0x01,M24512);
    snprintf(buf,sizeof buf,"op pos 0x01 van i2c staat %x\r\n",x);
    SERIAL_puts(buf);

    x = Read_24Cxx(0x01,M24512);
    snprintf(buf,sizeof buf,"op pos 0x01 van i2c staat %x\r\n",x);
    SERIAL_puts(buf);
    // uint8_t killswitch_val = Codec_ReadRegister(WM8731_LINVOL); /* read left input volume */
    int r = test_math();
    if(r == 0)
    {
        SERIAL_puts("MATH OK\r\n");
    }
    else
    {
        SERIAL_puts("MATH FAIL\r\n");
    }
    TimerHandle_t xSecondenTimer;
    xSecondenTimer = xTimerCreate("TimerSec", 1000, pdTRUE, (void *) 1, vTimerCallback);
    xTimerStart(xSecondenTimer, 0);
    vTaskStartScheduler(); /* will not return */
    while(1)
    {
        // we do not get here
    }
}


#ifdef USE_DEFAULT_TIMEOUT_CALLBACK

/**
  * @brief  Basic management of the timeout situation.
  * @param  None
  * @retval None
  */
uint32_t Codec_TIMEOUT_UserCallback(void)
{
    /* Block communication and all processes */
    while(1)
    {
    }
}

#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */
