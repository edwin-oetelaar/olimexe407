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

void init_LED_GPIO() {
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

// i2c PB8 en PB9
void init_I2C1(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    /* init the struct with defaults */
    GPIO_StructInit(&GPIO_InitStruct);
    I2C_StructInit(&I2C_InitStruct);
    /* enable gpio PB8 en PB9 as AF */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // internal pull up??
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* map AF pins to gpio */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    // enable clocks gpio
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


    /* Enable the CODEC_I2C peripheral clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* Reset I2C1 IP */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    int x = 100000;
    while (x) {
        x--;
    }
    /* Release reset signal of I2C1 IP */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    /* CODEC_I2C peripheral configuration */
    I2C_DeInit(I2C1);
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;

    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    /* Enable the I2C peripheral */

    I2C_Cmd(I2C1, ENABLE);

    I2C_Init(I2C1, &I2C_InitStruct);
}




void hardware_LED_Toggle() {
    static int i = 0;
    static uint32_t x = 0xFFFFFFFF - 10; // check overflow behaviour
    i = !i;
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, i); // RESET
    char buf[40];
    snprintf(buf, sizeof buf, "TimerCallback x=%lu\r\n", x++);
    SERIAL_puts(buf);
}

void vTimerCallback(void *ptr) {
    /* do something visible */
    hardware_LED_Toggle();
}


int main(void) {

    init_SERIAL(baudrate); // set up usart6
    RCC_ClocksTypeDef RCC_kloks;
    RCC_GetClocksFreq(&RCC_kloks);

    char buf[100];

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

    SERIAL_puts("USART6 init done\r\n");
    SERIAL_puts("GPIO init\r\n");
    init_LED_GPIO();
    SERIAL_puts("GPIO init done\r\n");


    SERIAL_puts("I2C init\r\n");
// we nemen PB9 en PB8 I2C1 omdat die op UEXT zitten
//   init_I2C1();
    SERIAL_puts("I2C init done\r\n");
    // uint8_t killswitch_val = Codec_ReadRegister(WM8731_LINVOL); /* read left input volume */
    int r = test_math();
    if (r == 0)
        SERIAL_puts("MATH OK\r\n");
    else
        SERIAL_puts("MATH FAIL\r\n");

    TimerHandle_t xSecondenTimer;

    xSecondenTimer = xTimerCreate("TimerSec", 500, pdTRUE, (void *) 1, vTimerCallback);

    xTimerStart(xSecondenTimer, 0);

    vTaskStartScheduler(); /* will not return */

    while (1) {
        // we do not get here
    }
}


#ifdef USE_DEFAULT_TIMEOUT_CALLBACK

/**
  * @brief  Basic management of the timeout situation.
  * @param  None
  * @retval None
  */
uint32_t Codec_TIMEOUT_UserCallback(void) {
    /* Block communication and all processes */
    while (1) {
    }
}

#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */
