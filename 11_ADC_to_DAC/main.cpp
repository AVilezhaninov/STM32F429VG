/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/* User */
#include "user\RCC.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define TIM6_PSC (90u - 1u)     /* TIM6 clock: (90 MHz / 90) = 1 MHz */
#define TIM6_ARR (1000u - 1u)   /* TIM6 IRQ period: (1 MHz / 1000) = 1 kHz */

#define TIM6_IRQ_PRIORITY   1u
#define ADC_IRQ_PRIORITY    1u

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitDac();
static void InitAdc();
static void InitTim6();

/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
extern "C" {

/**
 * TIM6 interrupt handler
 */
void TIM6_DAC_IRQHandler() {
  TIM6->SR &= ~TIM_SR_UIF;      /* Clear TIM6 update interrupt flag */
  ADC1->CR2 |= ADC_CR2_SWSTART; /* Start ADC conversion */
}

/**
 * ADC interrupt handler
 */
void ADC_IRQHandler() {
  uint16_t ADC_value = ADC1->DR; /* Get ADC value */

  DAC->DHR12R1 = ADC_value; /* Set DAC channel 1 to ADC value */
  DAC->DHR12R2 = ADC_value; /* Set DAC channel 2 to ADC value */
}

} /* extern "C" */

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main() {
  InitSystemClock();
  InitDac();
  InitAdc();
  InitTim6();

  while (1) {
    ;
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void InitDac() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* Enable PORTA clock */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;   /* Enable DAC clock */
  DAC->CR |= DAC_CR_EN1;               /* Enable DAC channel 1 */
  DAC->CR |= DAC_CR_EN2;               /* Enable DAC channel 2 */
}

static void InitAdc() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* Enable PORTA clock */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  /* Enable ADC clock */
  GPIOA->MODER |= GPIO_MODER_MODER3;   /* GPIO analog mode */
  ADC1->CR2 |= ADC_CR2_EOCS;           /* End of converdion selection */
  ADC1->SQR3 |= ADC_SQR3_SQ1_1 | ADC_SQR3_SQ1_0; /* Select 1st convertion
                                                  * in regular sequency
                                                  */
  ADC1->CR1 |= ADC_CR1_EOCIE;                    /* Enable EOC interrupt */

  NVIC_SetPriority(ADC_IRQn, ADC_IRQ_PRIORITY); /* Set ADC interrupt
                                                 * priority */
  NVIC_EnableIRQ(ADC_IRQn);                     /* Enable ADC interrupt */

  ADC1->CR2 |= ADC_CR2_ADON;    /* Turn ADC on */
}

static void InitTim6() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; /* Enable TIM6 clock */
  TIM6->PSC = TIM6_PSC;               /* Set TIM6 prescaler */
  TIM6->ARR = TIM6_ARR;               /* Set TIM6 auto reload value */
  TIM6->DIER |= TIM_DIER_UIE;         /* Enable TIM6 update interrupt */

  NVIC_SetPriority(TIM6_DAC_IRQn, TIM6_IRQ_PRIORITY); /* Set TIM6 interrupt
                                                       * priority
                                                       */
  NVIC_EnableIRQ(TIM6_DAC_IRQn);    /* Enable TIM6 interrupt */

  TIM6->CR1 |= TIM_CR1_CEN;         /* Enable TIM6 timer */
}
