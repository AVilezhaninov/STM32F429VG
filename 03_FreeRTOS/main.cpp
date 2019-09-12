/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* User */
#include "user\RCC.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define LED_TOGGLE_DELAY    500u

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitAll();
static void InitHardware();
static void InitGpio();
static void InitFreeRTOSObjects();
static void ToggleLed();

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
int main() {
  InitAll();
  vTaskStartScheduler();

  while (1) {
    ;
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void InitAll() {
  InitHardware();
  InitFreeRTOSObjects();
}

static void InitHardware() {
  InitSystemClock();
  InitGpio();
}

static void InitGpio() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  /* Enable GPIO clock */
  GPIOD->MODER |= GPIO_MODER_MODER14_0; /* PD14 in push-pull mode */
}

static void InitFreeRTOSObjects() {
  /* Create tasks */
  xTaskCreate(LedTask, "", configMINIMAL_STACK_SIZE, NULL, 1u, NULL);
}

static void ToggleLed() {
  GPIOD->ODR ^= GPIO_ODR_ODR_14; /* Toggle PD14 */
}
