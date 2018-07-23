/* CMSIS */
#include "stm32f4xx.h"

/* User */
#include "RCC.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
#define TIM2_PSC    (90u - 1u)
#define TIM2_ARR    (1000u - 1u)

#define TIM3_PSC    (90u - 1u)
#define TIM3_ARR    (1000u - 1u)

#define TIM4_PSC    (90u - 1u)
#define TIM4_ARR    (1000u - 1u)


/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
void TIM2_Init(void);
void TIM3_Init(void);
void TIM4_Init(void);


/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    /* Init system clock */
    SystemClock_Init();

    TIM2_Init();
    TIM3_Init();
    TIM4_Init();

    while (1u) {

    }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
void TIM2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* Enable GPIOA clock */
    GPIOA->MODER |= GPIO_MODER_MODER0_1;    /* GPIOA pin 0 in alternate mode */
    GPIOA->MODER |= GPIO_MODER_MODER1_1;    /* GPIOA pin 1 in alternate mode */
    GPIOA->MODER |= GPIO_MODER_MODER2_1;    /* GPIOA pin 2 in alternate mode */
    GPIOA->MODER |= GPIO_MODER_MODER3_1;    /* GPIOA pin 3 in alternate mode */
    GPIOA->AFR[0u] |= (1u << 0u);           /* PA0 in TIM2_CH1 AF2 */
    GPIOA->AFR[0u] |= (1u << 4u);           /* PA1 in TIM2_CH2 AF2 */
    GPIOA->AFR[0u] |= (1u << 8u);           /* PA2 in TIM2_CH3 AF2 */
    GPIOA->AFR[0u] |= (1u << 12u);          /* PA3 in TIM2_CH4 AF2 */

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     /* Enable TIM2 clock */
    TIM2->PSC = TIM2_PSC;                   /* Set TIM2 prescaler */
    TIM2->ARR = TIM2_ARR;                   /* Set TIM2 auto reload value */
    TIM2->CCR1 = TIM2->ARR / 2u;            /* Set TIM2 CH1 compare value */
    TIM2->CCR2 = TIM2->ARR / 2u;            /* Set TIM2 CH2 compare value */
    TIM2->CCR3 = TIM2->ARR / 2u;            /* Set TIM2 CH3 compare value */
    TIM2->CCR4 = TIM2->ARR / 2u;            /* Set TIM2 CH4 compare value */
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; /* Set CH1 PMW mode 1 */
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; /* Set CH2 PMW mode 1 */
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; /* Set CH3 PMW mode 1 */
    TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; /* Set CH4 PMW mode 1 */
    TIM2->CCER |= TIM_CCER_CC1E;            /* Enable CH1 compare output */
    TIM2->CCER |= TIM_CCER_CC2E;            /* Enable CH2 compare output */
    TIM2->CCER |= TIM_CCER_CC3E;            /* Enable CH3 compare output */
    TIM2->CCER |= TIM_CCER_CC4E;            /* Enable CH4 compare output */
    TIM2->CR1 |= TIM_CR1_CEN;               /* Enable TIM2 */
}


void TIM3_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    /* Enable GPIOC clock */
    GPIOC->MODER |= GPIO_MODER_MODER6_1;    /* GPIOC pin 6 in alternate mode */
    GPIOC->MODER |= GPIO_MODER_MODER7_1;    /* GPIOC pin 7 in alternate mode */
    GPIOC->MODER |= GPIO_MODER_MODER8_1;    /* GPIOC pin 8 in alternate mode */
    GPIOC->MODER |= GPIO_MODER_MODER9_1;    /* GPIOC pin 9 in alternate mode */
    GPIOC->AFR[0u] |= (2u << 24u);          /* PC6 in TIM3_CH1 AF2 */
    GPIOC->AFR[0u] |= (2u << 28u);          /* PC7 in TIM3_CH2 AF2 */
    GPIOC->AFR[1u] |= (2u << 0u);           /* PC8 in TIM3_CH3 AF2 */
    GPIOC->AFR[1u] |= (2u << 4u);           /* PC9 in TIM3_CH4 AF2 */

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     /* Enable TIM3 clock */
    TIM3->PSC = TIM3_PSC;                   /* Set TIM3 prescaler */
    TIM3->ARR = TIM3_ARR;                   /* Set TIM3 auto reload value */
    TIM3->CCR1 = TIM3->ARR / 2u;            /* Set TIM3 CH1 compare value */
    TIM3->CCR2 = TIM3->ARR / 2u;            /* Set TIM3 CH2 compare value */
    TIM3->CCR3 = TIM3->ARR / 2u;            /* Set TIM3 CH3 compare value */
    TIM3->CCR4 = TIM3->ARR / 2u;            /* Set TIM3 CH4 compare value */
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; /* Set CH1 PMW mode 1 */
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; /* Set CH2 PMW mode 1 */
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; /* Set CH3 PMW mode 1 */
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; /* Set CH4 PMW mode 1 */
    TIM3->CCER |= TIM_CCER_CC1E;            /* Enable CH1 compare output */
    TIM3->CCER |= TIM_CCER_CC2E;            /* Enable CH2 compare output */
    TIM3->CCER |= TIM_CCER_CC3E;            /* Enable CH3 compare output */
    TIM3->CCER |= TIM_CCER_CC4E;            /* Enable CH4 compare output */
    TIM3->CR1 |= TIM_CR1_CEN;               /* Enable TIM3 */
}


void TIM4_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    /* Enable GPIOB clock */
    GPIOB->MODER |= GPIO_MODER_MODER6_1;    /* GPIOB pin 6 in alternate mode */
    GPIOB->MODER |= GPIO_MODER_MODER7_1;    /* GPIOB pin 7 in alternate mode */
    GPIOB->MODER |= GPIO_MODER_MODER8_1;    /* GPIOB pin 8 in alternate mode */
    GPIOB->MODER |= GPIO_MODER_MODER9_1;    /* GPIOB pin 9 in alternate mode */
    GPIOB->AFR[0u] |= (2u << 24u);          /* PB6 in TIM4_CH1 AF2 */
    GPIOB->AFR[0u] |= (2u << 28u);          /* PB7 in TIM4_CH2 AF2 */
    GPIOB->AFR[1u] |= (2u << 0u);           /* PB8 in TIM4_CH3 AF2 */
    GPIOB->AFR[1u] |= (2u << 4u);           /* PB9 in TIM4_CH4 AF2 */

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;     /* Enable TIM4 clock */
    TIM4->PSC = TIM4_PSC;                   /* Set TIM4 prescaler */
    TIM4->ARR = TIM4_ARR;                   /* Set TIM4 auto reload value */
    TIM4->CCR1 = TIM4->ARR / 2u;            /* Set TIM4 CH1 compare value */
    TIM4->CCR2 = TIM4->ARR / 2u;            /* Set TIM4 CH2 compare value */
    TIM4->CCR3 = TIM4->ARR / 2u;            /* Set TIM4 CH3 compare value */
    TIM4->CCR4 = TIM4->ARR / 2u;            /* Set TIM4 CH4 compare value */
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; /* Set CH1 PMW mode 1 */
    TIM4->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; /* Set CH2 PMW mode 1 */
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; /* Set CH3 PMW mode 1 */
    TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; /* Set CH4 PMW mode 1 */
    TIM4->CCER |= TIM_CCER_CC1E;            /* Enable CH1 compare output */
    TIM4->CCER |= TIM_CCER_CC2E;            /* Enable CH2 compare output */
    TIM4->CCER |= TIM_CCER_CC3E;            /* Enable CH3 compare output */
    TIM4->CCER |= TIM_CCER_CC4E;            /* Enable CH4 compare output */
    TIM4->CR1 |= TIM_CR1_CEN;               /* Enable TIM4 */
}
