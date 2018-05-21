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
#define START_TASK_DELAY_MS     500u
#define LED_TOGGLE_DELAY_MS     200u
#define IWDG_TEST_DELAY_MS      2000u


/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void IWDG_Init(const uint16_t IWDG_period);
static void IWDG_Reset(void);


/******************************************************************************/
/* Tasks **********************************************************************/
/******************************************************************************/
void LEDBlinkTask(void *pvParameters) {
    (void)pvParameters;
    uint8_t task_cnt = 0u;

    vTaskDelay(START_TASK_DELAY_MS);

    while (1) {
        if (task_cnt < (IWDG_TEST_DELAY_MS / LED_TOGGLE_DELAY_MS)) {
            ++task_cnt;
            IWDG_Reset();
        }

        GPIOD->ODR ^= GPIO_ODR_ODR_14;      /* Toggle GPIOD pin 14 */
        vTaskDelay(LED_TOGGLE_DELAY_MS);
    }
}


/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    /* Init system clock */
    SystemClock_Init();

    /* Init IWDG timer for 1 second period */
    IWDG_Init(1000u);

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


/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void IWDG_Init(const uint16_t IWDG_period) {
    IWDG_Reset();              /* reset independent watch dog */
    IWDG->KR = 0x5555u;        /* enable access */
    IWDG->PR = IWDG_PR_PR_0;   /* divider 8 */
    if (IWDG_period <= 1000u) {
        IWDG->RLR = (uint16_t)(IWDG_period * 4u);
    } else {
        IWDG->RLR = 4000u;
    }
    IWDG->KR = 0xCCCCu;   /* start */
    IWDG_Reset();         /* reset independent watch dog */
}


static void IWDG_Reset(void) {
    IWDG->KR = 0xAAAAu;
}
