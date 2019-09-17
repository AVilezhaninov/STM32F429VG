/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/* User */
#include "user\RCC.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define TIM2_PSC    (9000u - 1u)  /* 10kHz timer clock */
#define TIM2_ARR    (1000u - 1u)  /* 100ms ADC start period */

#define ADC_IRQ_PRIORITY            5u
#define INTERNAL_REFERENCE_VOLTAGE  3.0f
#define MAX_ADC_VALUE               4095u

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
void InitGpios();
void InitTimer();
void InitAdc();
void StartConversion();

/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
extern "C" {

void ADC_IRQHandler(void) {
  uint16_t ch0_value = ADC1->DR;
  float ch0_voltage = ch0_value * (INTERNAL_REFERENCE_VOLTAGE / MAX_ADC_VALUE);

  ADC1->SR &= ~ADC_SR_EOC;
}

} /* extern "C" */

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
  InitSystemClock();
  InitGpios();
  InitTimer();
  InitAdc();
  StartConversion();

  while (1) {
    ;
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
void InitGpios() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  /* Enable port clock */
  GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE0_0; /* PA0 in analog
                                                            *  mode */
}

void InitTimer() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* Enable timer clock */
  TIM2->PSC = TIM2_PSC;               /* Set prescaler */
  TIM2->ARR = TIM2_ARR;               /* Set auto-reload value */
  TIM2->DIER |= TIM_DIER_UIE;         /* Enable update IRQ */
  TIM2->CR2 |= TIM_CR2_MMS_1;         /* Update TRGO output */
}

void InitAdc() {
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* Enable ADC clock */
  ADC1->SQR3 &= ~ADC_SQR3_SQ1;        /* Ch0 */
  ADC1->SMPR2 |= ADC_SMPR2_SMP0;      /* Ch0 -  480 cycles conversion */
  ADC1->CR1 |= ADC_CR1_EOCIE;         /* Enable EOC interrupt */
  ADC1->CR2 |= ADC_CR2_EXTEN;         /* Enable external trigger */
  ADC1->CR2 |= ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1; /* Timer 2 TRGO event */
  ADC->CCR |= ADC_CCR_ADCPRE_0;       /* ADC prescaler - 4 */

  NVIC_SetPriority(ADC_IRQn, ADC_IRQ_PRIORITY); /* Set ADC IRQ priority */
  NVIC_EnableIRQ(ADC_IRQn);                     /* Enable ADC interrupt */
}

void StartConversion() {
  ADC1->CR2 |= ADC_CR2_ADON; /* Turn ADC on */
  TIM2->CR1 |= TIM_CR1_CEN;  /* Turn on ADC timer */
}
