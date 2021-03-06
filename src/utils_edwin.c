#include <stm32f4xx_gpio.h>
#include "utils_edwin.h"
#include "FreeRTOS.h"
#include "task.h"

/* TX buffer for serial USART */
struct {
    queue_hdr_t hdr; // must be named "hdr"
    uint8_t items[256]; // must be named "items", 1 space wasted
} my_TX_queue;

/* RX buffer for serial USART */
struct {
    queue_hdr_t hdr; // must be named "hdr"
    uint8_t items[128]; // must be named "items", 1 space wasted
} my_RX_queue;


/* debug info on USART 6 connected to USB */
static void init_USART6(uint32_t baudrate) {
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
    USART_InitTypeDef USART_InitStruct; // this is for the USART2 initialization
    NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
    /* init the struct with defaults */
    GPIO_StructInit(&GPIO_InitStruct);
    USART_StructInit(&USART_InitStruct);
    // enable Clocks for APB2 and GPIOC
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // Connect to pins to AF (must be first??)
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    // init the pins as alternate function
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    /* enable clock on usart */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    /* setup the usart parameters */
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    /* write config into usart */
    USART_Init(USART6, &USART_InitStruct);
// enable the USART2 receive interrupt
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    //disable Transmit Data Register empty interrupt
    USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
    // setup the interrupt controller
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;//f;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f; //f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // write config into registers
    NVIC_Init(&NVIC_InitStructure);
//Enable USART6
    USART_Cmd(USART6, ENABLE);
}

// this is the interrupt request handler (IRQ) for ALL USART6 interrupts
void USART6_IRQHandler(void) {
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
        }
        else {
            // no data in buf, disable Transmit Data Register empty interrupt
            USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
        }
    }
    // Clear all flags
    // USART_ClearITPendingBit(USART6, (USART_IT_CTS | USART_IT_LBD | USART_IT_TC | USART_IT_RXNE));
}

void init_SERIAL(uint32_t baudrate) {
    QUEUE_INIT(my_TX_queue); // usart6 serial
    QUEUE_INIT(my_RX_queue); // usart6 serial
    init_USART6(baudrate);
}

/* The fault handler implementation calls a function called prvGetRegistersFromStack(). */

static void HardFault_Handler(void) {
    __asm volatile
    (
    " tst lr, #4                                                \n"
            " ite eq                                                    \n"
            " mrseq r0, msp                                             \n"
            " mrsne r0, psp                                             \n"
            " ldr r1, [r0, #24]                                         \n"
            " ldr r2, handler2_address_const                            \n"
            " bx r2                                                     \n"
            " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}


/* called when stack is about to crash */

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName) {
    /* hang here so we can attach a debugger to find out what happened */
    for (; ;);
}


void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress) {
    /* These are volatile to try and prevent the compiler/linker optimising them
    away as the variables never actually get used.  If the debugger won't show the
    values of the variables, make them global my moving their declaration outside
    of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr; /* Link register. */
    volatile uint32_t pc; /* Program counter. */
    volatile uint32_t psr;/* Program status register. */
    r0 = pulFaultStackAddress[0];
    r1 = pulFaultStackAddress[1];
    r2 = pulFaultStackAddress[2];
    r3 = pulFaultStackAddress[3];
    r12 = pulFaultStackAddress[4];
    lr = pulFaultStackAddress[5];
    pc = pulFaultStackAddress[6];
    psr = pulFaultStackAddress[7];
    /* When the following line is hit, the variables contain the register values. */
    for (; ;);
}


/* stop een string in de output buffer van de serial poort
de interrupt handler zorgt ervoor dat de buffer vanzelf
verstuurd wordt zonder dat dit tijd kost in het programma
Als de buffer vol is worden de tekens zomaar weggegooid, jammer dan.
 */
void SERIAL_puts(const char *s) {
    while (*s) {
        if (!QUEUE_FULL(my_TX_queue)) {
            // er is ruimte, stop teken in de queue
            QUEUE_PUT(my_TX_queue, *s);
            // volgende teken
            s++;
        }
        else {
            // buffer vol, we moeten wat anders doen nu
            // deze functie blockt nu
            USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
            vTaskDelay(0);
        }
    }
    // de interrupt handler voor Transmit register Empty aanzetten
    USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
}

void SERIAL_write(const char *s, uint32_t len) {
    while (len) {
        if (!QUEUE_FULL(my_TX_queue)) {
            // er is ruimte, stop teken in de queue
            QUEUE_PUT(my_TX_queue, *s);
            // volgende teken
            s++;
            len--;
        }
        else {
            // buffer vol, we moeten wat anders doen nu
            // deze functie blockt nu
            USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
            vTaskDelay(0);
        }
    }
    // de interrupt handler voor Transmit register Empty aanzetten
    USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
}


// i2c PB8 en PB9
void init_I2C1(void) {
    // PB8 en PB9 as i2c
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;
    /* clock on i2c */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* clock on GPIOB */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* init the struct with defaults */
    GPIO_StructInit(&GPIO_InitStruct);
    /* map AF pins to gpio */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    /* enable gpio PB8 en PB9 as AF */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // internal pull up??
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // enable clocks gpio
#if 0
    /* Reset I2C1 IP */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    int x = 100000;
    while(x)
    {
        x--;
    }
    /* Release reset signal of I2C1 IP */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
#endif
    /* CODEC_I2C peripheral configuration */
    I2C_DeInit(I2C1);
    /*I2C stuff */
    I2C_StructInit(&I2C_InitStruct);
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x33;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = 1000;
    /* set up the registers */
    I2C_Init(I2C1, &I2C_InitStruct);
    /* Enable the I2C peripheral LAST */
    I2C_Cmd(I2C1, ENABLE);

}


uint8_t Write_24Cxx(uint16_t Addr, uint8_t Data, uint8_t Mem_Type) {
    uint32_t timeout;
    uint8_t upper_addr;
    uint8_t lower_addr;

    lower_addr = (uint8_t) ((0x00FF) & Addr);

    if (Mem_Type >= M2432) {
        Addr = Addr >> 8;
        upper_addr = (uint8_t) ((0x00FF) & Addr);
    }
    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on I2C1 EV5, Start trnsmitted successfully and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFF;
        }
    }
    /* Send Memory device slave Address for write */
    I2C_Send7bitAddress(I2C1, MEM_DEVICE_WRITE_ADDR, I2C_Direction_Transmitter);
    /* Test on I2C1 EV6 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFF;
        }
    }

    if (Mem_Type >= M2432) {
        /* Send I2C1 location address LSB */
        I2C_SendData(I2C1, upper_addr);
        /* Test on I2C1 EV8 and clear it */
        timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            /* If the timeout delay is exeeded, exit with error code */
            if ((timeout--) == 0) {
                return 0xFF;
            }
        }
    }
    /* Send I2C1 location address LSB */
    I2C_SendData(I2C1, lower_addr);
    /* Test on I2C1 EV8 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFF;
        }
    }
    /* Send Data */
    I2C_SendData(I2C1, Data);
    /* Test on I2C1 EV8 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFF;
        }
    }
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    /* If operation is OK, return 0 */
    return 0;
}


uint8_t Busy_24Cxx(void) {
    /* poll the memory chip after a write, to see if it is still busy
     * we do this by initiation of a write command on i2c address
     * if not Acked, it is still busy working
     * */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on I2C1 EV5 and clear it */
    uint32_t timeout = 10000; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFE; // error
        }
    }
    /* slave Address for write, as by http://www.ece.msstate.edu/~reese/ece3724/lectures/chap11_i2c.pdf */
    I2C_Send7bitAddress(I2C1, MEM_DEVICE_WRITE_ADDR, I2C_Direction_Transmitter);
    /* Test on I2C1 EV6 and clear it */
    timeout = 10000; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            I2C_GenerateSTOP(I2C1, ENABLE);
            return 0xF8; // busy
        }
    }
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0; // not busy
}

uint8_t Read_24Cxx(uint16_t Addr, uint8_t Mem_Type) {
    uint32_t timeout = I2C_TIMEOUT_MAX;
    uint8_t Data = 0;
    uint8_t upper_addr, lower_addr;
    lower_addr = (uint8_t) ((0x00FF) & Addr);
    if (Mem_Type >= M2432) {
        Addr = Addr >> 8;
        upper_addr = (uint8_t) ((0x00FF) & Addr);
    }
    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on I2C1 EV5 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFE;
        }
    }
    /* Send DCMI selcted device slave Address for write */
    I2C_Send7bitAddress(I2C1, MEM_DEVICE_WRITE_ADDR, I2C_Direction_Transmitter);
    /* Test on I2C1 EV6 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xF8;
        }
    }
    if (Mem_Type >= M2432) {
        /* Send I2C1 location address LSB */
        I2C_SendData(I2C1, upper_addr);
        /* Test on I2C1 EV8 and clear it */
        timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            /* If the timeout delay is exeeded, exit with error code */
            if ((timeout--) == 0) {
                return 0xF9;
            }
        }
    }
    /* Send I2C1 location address LSB */
    I2C_SendData(I2C1, lower_addr);
    /* Test on I2C1 EV8 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFA;
        }
    }
    /* Clear AF flag if arised */
    //I2C1->SR1 |= (uint16_t)0x0400;
    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on I2C1 EV6 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFB;
        }
    }
    /* Send DCMI selcted device slave Address for write */
    I2C_Send7bitAddress(I2C1, MEM_DEVICE_READ_ADDR, I2C_Direction_Receiver);
    /* Test on I2C1 EV6 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) {
            return 0xFc;
        }
    }
    /* Prepare an NACK for the next data received */
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    /* Prepare Stop after receiving data */
    I2C_GenerateSTOP(I2C1, ENABLE);
    /* Test on I2C1 EV7 and clear it */
    timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        /* If the timeout delay is exeeded, exit with error code */
        //while(1);
        if ((timeout--) == 0) {
            return 0xFe;
        }
    }

    /* Receive the Data */
    Data = I2C_ReceiveData(I2C1);
    /* return the read data */
    return Data;
}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif
