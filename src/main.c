/*
 * made by Edwin van den Oetelaar
 * No warranty whatsoever
 * made for personal use, but if you like it, use it
*/

#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"

/* called when stack is about to crash */
void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName)
{
    /* hang here so we can attach a debugger to find out what happened */
    for (; ;);
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


void hardware_init_LED_GPIO()
{
// enable clocks
// enable pin
    GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO
    // enable Clocks for APB1 and GPIOB en C
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // de pinnen van de SPI-2 instellen
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 ;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

    GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void hardware_LED_Toggle()
{
    static int i = 0;
    i = !i;
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, i); // RESET

}
void vTimerCallback(void *ptr)
{
    /* do something visible */
    hardware_LED_Toggle();
}

int main(void)
{

    hardware_init_LED_GPIO();

    TimerHandle_t xSecondenTimer;

    xSecondenTimer = xTimerCreate("TimerSec", 500, pdTRUE, (void *) 1, vTimerCallback);

    xTimerStart(xSecondenTimer, 0);

    vTaskStartScheduler(); /* will not return */

    while (1) {
        // we do not get here
    }
}



