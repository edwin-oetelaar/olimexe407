/*
 * made by Edwin van den Oetelaar
 * No warranty whatsoever
 * made for personal use, but if you like it, use it
*/

#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "utils_edwin.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"
#include "my_queue.h"
#include <stdio.h>

const uint32_t baudrate = 115200 ; // debug console baud rate

struct {
    queue_hdr_t hdr; // must be named "hdr"
    uint8_t items[256]; // must be named "items", 1 space wasted
} my_TX_queue;

struct {
    queue_hdr_t hdr; // must be named "hdr"
    uint8_t items[128]; // must be named "items", 1 space wasted
} my_RX_queue;

/* stop een string in de output buffer van de serial poort
de interrupt handler zorgt ervoor dat de buffer vanzelf
verstuurd wordt zonder dat dit tijd kost in het programma
Als de buffer vol is worden de tekens zomaar weggegooid, jammer dan.
 */
void SERIAL_puts(const char *s)
{
    while (*s) {
        if (!QUEUE_FULL(my_TX_queue)) {
            // er is ruimte, stop teken in de queue
            QUEUE_PUT(my_TX_queue, *s);
            // volgende teken
            s++;
        } else {
            // buffer vol, we moeten wat anders doen nu
            // deze functie blockt nu
            USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
            vTaskDelay(0);
        }
    }
    // de interrupt handler voor Transmit register Empty aanzetten
    USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
}

void SERIAL_write(const char *s, uint32_t len)
{
    while (len) {
        if (!QUEUE_FULL(my_TX_queue)) {
            // er is ruimte, stop teken in de queue
            QUEUE_PUT(my_TX_queue, *s);
            // volgende teken
            s++;
            len--;
        } else {
            // buffer vol, we moeten wat anders doen nu
            // deze functie blockt nu
            USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
            vTaskDelay(0);
        }
    }
    // de interrupt handler voor Transmit register Empty aanzetten
    USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
}

void init_LED_GPIO()
{
// enable clocks
// enable pin
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO
    // enable Clocks for APB1 and GPIOB en C
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // de pinnen van de LED instellen
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 ;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

    GPIO_Init(GPIOC, &GPIO_InitStruct);

}


/* debug info on USART 6 connected to USB */
void init_USART6(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
    USART_InitTypeDef USART_InitStruct; // this is for the USART2 initialization
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

    /* init the struct with defaults */
    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);

    // enable Clocks for APB1 and GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    //Connect to AF
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); // PA2 tx
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); // PA3 rx

    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART6, &USART_InitStruct);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt
    //disable Transmit Data Register empty interrupt
    USART_ITConfig(USART6, USART_IT_TXE, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART6, ENABLE); //Enable USART2
}

// i2c PB8 en PB9
void init_I2C1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    /* init the struct with defaults */
    GPIO_StructInit(&GPIO_InitStruct);
    I2C_StructInit(&I2C_InitStruct);
    /* enable gpio PB8 en PB9 as AF */
    GPIO_InitStruct.GPIO_Pin =GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStruct);
    /* map AF pins to gpio */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    // enable clocks gpio
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);


    /* Enable the CODEC_I2C peripheral clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* CODEC_I2C peripheral configuration */
    I2C_DeInit(I2C1);
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x33;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    /* Enable the I2C peripheral */
    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStruct);



}


// this is the interrupt request handler (IRQ) for ALL USART6 interrupts
void USART6_IRQHandler(void)
{
    /* if Receive Register not empty then get byte */
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART6->DR;
        // als de buffer niet vol dan erbij proppen, anders jammer dan, weg ermee
        if (!QUEUE_FULL(my_RX_queue)) {
            QUEUE_PUT(my_RX_queue, ch);
        }
    }
    // moeten we wat zenden, TX empty interrupt?
    if (USART_GetITStatus(USART6, USART_IT_TXE) != RESET) {
        uint8_t ch;
        if (!QUEUE_EMPTY(my_TX_queue)) {
            QUEUE_GET(my_TX_queue, ch);
            USART_SendData(USART6, ch);
        } else {
            // no data in buf, disable Transmit Data Register empty interrupt
            USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
        }
    }
    // Clear all flags
    // USART_ClearITPendingBit(USART6, (USART_IT_CTS | USART_IT_LBD | USART_IT_TC | USART_IT_RXNE));
}

void hardware_LED_Toggle()
{
    static int i = 0;
    static uint32_t x = 0xFFFFFFFF - 10; // check overflow behaviour
    i = !i;
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, i); // RESET
    char buf[20];
    snprintf(buf,sizeof (buf) / sizeof(buf[0]),"x=%lu\r\n",x++);
    SERIAL_puts(buf);
}

void vTimerCallback(void *ptr)
{
    /* do something visible */
    hardware_LED_Toggle();
}

int main(void)
{
    QUEUE_INIT(my_TX_queue); // usart6 serial
    QUEUE_INIT(my_RX_queue); // usart6 serial

    init_USART6(baudrate);
    SERIAL_puts("USART6 init done\r\n");
    SERIAL_puts("GPIO init\r\n");
    init_LED_GPIO();
    SERIAL_puts("GPIO init done\r\n");


    SERIAL_puts("I2C init\r\n");
// we nemen PB9 en PB8 I2C1 omdat die op UEXT zitten
    init_I2C1();
    SERIAL_puts("I2C init done\r\n");


    TimerHandle_t xSecondenTimer;

    xSecondenTimer = xTimerCreate("TimerSec", 500, pdTRUE, (void *) 1, vTimerCallback);

    xTimerStart(xSecondenTimer, 0);

    vTaskStartScheduler(); /* will not return */

    while (1) {
        // we do not get here
    }
}



