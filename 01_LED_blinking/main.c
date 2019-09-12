/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define LED_TOGGLE_DELAY  1000000u

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitGpio(void);
static void ToggleLed(void);
static void DummyDelay(const int delay);

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
  InitGpio();

  while (1) {
    ToggleLed();
    DummyDelay(LED_TOGGLE_DELAY);
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void InitGpio(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  /* Enable GPIO clock */
  GPIOD->MODER |= GPIO_MODER_MODER14_0; /* GPIOD pin 14 in push-pull mode */
}

static void ToggleLed(void) {
  GPIOD->ODR ^= GPIO_ODR_ODR_14;        /* Toggle GPIOD pin 14 */
}

static void DummyDelay(const int delay) {
  for (volatile int i = 0; i < delay; ++i) {
    ;
  }
}
