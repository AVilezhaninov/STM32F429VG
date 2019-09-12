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
#define START_TASK_DELAY_MS   500u
#define LED_TOGGLE_DELAY_MS   200u
#define IWDG_TEST_DELAY_MS    2000u

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitAll();
static void InitHardware();
static void InitRTOSObjects();
static void InitIwdg(const uint16_t iwdg_period);
static void ResetIwdgt();
static void ToggleLed();

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
      ResetIwdgt();
    }

    ToggleLed();
    vTaskDelay(LED_TOGGLE_DELAY_MS);
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
  InitRTOSObjects();
}

static void InitHardware() {
  InitSystemClock();
  InitIwdg(1000u);  /* Init IWDG timer for 1 second period */

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  /* Enable GPIO clock */
  GPIOD->MODER |= GPIO_MODER_MODER14_0; /* GPIOD pin 14 in push-pull mode */
}

static void InitRTOSObjects() {
  /* Create tasks */
  xTaskCreate(LEDBlinkTask, "", configMINIMAL_STACK_SIZE, NULL, 1u, NULL);
}

static void InitIwdg(const uint16_t iwdg_period) {
  ResetIwdgt();            /* Reset independent watch dog */
  IWDG->KR = 0x5555u;      /* Enable access */
  IWDG->PR = IWDG_PR_PR_0; /* Divider 8 */
  if (iwdg_period <= 1000u) {
    IWDG->RLR = (uint16_t)(iwdg_period * 4u);
  } else {
    IWDG->RLR = 4000u;
  }
  IWDG->KR = 0xCCCCu;     /* Start IWDG */
  ResetIwdgt();
}

static void ResetIwdgt() {
  IWDG->KR = 0xAAAAu;
}

static void ToggleLed() {
  GPIOD->ODR ^= GPIO_ODR_ODR_14;  /* Toggle GPIOD pin 14 */
}
