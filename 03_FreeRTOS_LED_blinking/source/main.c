/* CMSIS */
#include "stm32f4xx.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* User */
#include "RCC.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
#define LED_TOGGLE_DELAY_MS     500u


/******************************************************************************/
/* Tasks **********************************************************************/
/******************************************************************************/
void LEDBlinkTask(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        /* Toggle GPIOD pin 14 */
        GPIOD->ODR ^= GPIO_ODR_ODR_14;
        /* Task delay */
        vTaskDelay(LED_TOGGLE_DELAY_MS);
    }
}


/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    /* Init system clock */
    SystemClock_Init();

    /* Enable GPIOD clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    /* GPIOD pin 14 in push-pull mode */
    GPIOD->MODER |= GPIO_MODER_MODER14_0;

    /* FreeRTOS init */
    /* Create LED blinking task */
    xTaskCreate(LEDBlinkTask, (char const*)"", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1u, NULL);
    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

    while (1u) {

    }
}
