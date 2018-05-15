/* CMSIS */
#include "stm32f4xx.h"

/* User */
#include "RCC.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
/* TIM6 prescaler and auto reload values */
#define TIM6_PSC            8999u /* TIM6 clock: (90 MHz / 9000) = 10 kHz */
#define TIM6_ARR            4999u /* TIM6 IRQ period: (10 kHz / 5000) = 2 Hz */
#define TIM6_IRQ_PRIORITY   1u      


/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
/**
 * TIM6 interrupt handler
 */
void TIM6_DAC_IRQHandler(void) {
    /* Clear TIM6 update interrupt flag */
    TIM6->SR &= ~TIM_SR_UIF;
    /* Toggle LED */
    GPIOD->ODR ^= GPIO_ODR_ODR_14;
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

    /* Enable TIM6 clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    /* Set TIM6 prescaler */
    TIM6->PSC = TIM6_PSC;
    /* Set TIM6 auto reload value */
    TIM6->ARR = TIM6_ARR;
    /* Enable TIM6 update interrupt */
    TIM6->DIER |= TIM_DIER_UIE;
    /* Set TIM6 interrupt priority */
    NVIC_SetPriority(TIM6_DAC_IRQn, TIM6_IRQ_PRIORITY);
    /* Enable TIM6 interrupt */
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    /* Enable TIM6 timer */
    TIM6->CR1 |= TIM_CR1_CEN;

    while (1u) {

    }
}
