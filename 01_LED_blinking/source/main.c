/* CMSIS */
#include "stm32f4xx.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
#define LED_TOGGLE_DELAY    1000000u


/******************************************************************************/
/* Static function prototypes *************************************************/
/******************************************************************************/
static void GPIO_Init(void);
static void ToggleLed(void);
static void DummyDelay(const int delay);




/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    GPIO_Init();

    while (1) {
        ToggleLed();
        DummyDelay(LED_TOGGLE_DELAY);
    }
}




/******************************************************************************/
/* Static functions ***********************************************************/
/******************************************************************************/
static void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;    /* Enable GPIOD clock */
    GPIOD->MODER |= GPIO_MODER_MODER14_0;   /* GPIOD pin 14 in push-pull mode */
}


static void ToggleLed(void) {
    GPIOD->ODR ^= GPIO_ODR_ODR_14;          /* Toggle GPIOD pin 14 */
}


static void DummyDelay(const int delay) {
    for (volatile int i = 0; i < delay; ++i) {
        ;
    }
}
