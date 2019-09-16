/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/* User */
#include "user\RCC.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define TIM1_PSC            0u     /* TIM1 clock: (180 MHz / 1) = 180 MHz */
#define TIM1_ARR            65535u /* TIM1 maximum clock value */
#define TIM1_IRQ_PRIORITY   5u

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitTim1();

/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
extern "C" {

/**
 * TIM1 capture compare interrupt handler
 */
void TIM1_CC_IRQHandler() {
  uint16_t period;
  uint16_t width;

  if ((TIM1->SR & TIM_SR_CC2IF) == TIM_SR_CC2IF) {
    period = TIM1->CCR2;      /* Get pulse period */
    width = TIM1->CCR1;       /* Get pulse width */
  }
  TIM1->SR &= ~TIM_SR_CC1OF;  /* Clear overcapture 1 flag */
  TIM1->SR &= ~TIM_SR_CC2OF;  /* Clear overcapture 2 flag */
}

} /* extern "C" */

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
  InitSystemClock();
  InitTim1();

  while (1) {
    ;
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
void InitTim1() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;          /* Enable PORTE clock */
  GPIOE->MODER |= GPIO_MODER_MODER11_1;         /* PE11 alternate mode */
  GPIOE->AFR[1u] |= (1u << 12u);                /* PE11 in AF1 */

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;           /* Enable TIM1 clock */
  TIM1->PSC = TIM1_PSC;                         /* Set TIM1 prescaler */
  TIM1->ARR = TIM1_ARR;                         /* Set TIM1 auto reload value */
  TIM1->CCMR1 |= TIM_CCMR1_CC1S_1;              /* IC1 mapped on TI2 */
  TIM1->CCER |= TIM_CCER_CC1P;                  /* Falling edge on TI1 */
  TIM1->CCMR1 |= TIM_CCMR1_CC2S_0;              /* IC2 mapped on TI1 */
  TIM1->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_1;  /* Filtered timer input 2 */
  TIM1->SMCR |= TIM_SMCR_SMS_2;                 /* "Reset" slave mode */
  TIM1->DIER |= TIM_DIER_CC2IE;                 /* Capture 2 interrupt enable */
  TIM1->CCER |= TIM_CCER_CC2E;                  /* Caputre 2 output enbale */
  TIM1->CCER |= TIM_CCER_CC1E;                  /* Caputre 1 output enbale */

  NVIC_SetPriority(TIM1_CC_IRQn, TIM1_IRQ_PRIORITY); /* Set TIM1 interrupt
                                                      * priority */
  NVIC_EnableIRQ(TIM1_CC_IRQn); /* Enable TIM1 capture interrupt */

  TIM1->CR1 |= TIM_CR1_CEN;     /* Enable TIM1 timer */
}
