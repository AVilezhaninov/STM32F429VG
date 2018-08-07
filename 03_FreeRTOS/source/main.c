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
#define LED_TOGGLE_DELAY    500u


/******************************************************************************/
/* Static function prototypes *************************************************/
/******************************************************************************/
static void InitAll(void);
static void InitHardware(void);
static void GPIO_Init(void);
static void InitFreeRTOSObjects(void);
static void ToggleLed(void);


/******************************************************************************/
/* Tasks **********************************************************************/
/******************************************************************************/
void LedTask(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        ToggleLed();
        vTaskDelay(LED_TOGGLE_DELAY);
    }
}




/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    InitAll();
    vTaskStartScheduler();

    while (1u) {

    }
}




/******************************************************************************/
/* Static functions ***********************************************************/
/******************************************************************************/
static void InitAll(void) {
    InitHardware();
    InitFreeRTOSObjects();
}


static void InitHardware(void) {
    SystemClock_Init();
    GPIO_Init();
}


static void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;    /* Enable GPIOD clock */
    GPIOD->MODER |= GPIO_MODER_MODER14_0;   /* PD14 in push-pull mode */
}


static void InitFreeRTOSObjects(void) {
    /* Create RTOS tasks */
    xTaskCreate(LedTask, "", configMINIMAL_STACK_SIZE, NULL, 1u, NULL);
}


static void ToggleLed(void) {
    GPIOD->ODR ^= GPIO_ODR_ODR_14;        /* Toggle PD14 */
}
