/* CMSIS */
#include "stm32f4xx.h"

/* User */
#include "RCC.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
#define USART1_BAUDRATE         115200u
#define USART1_IRQ_PRIORITY     5u


/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void USART1_Init(void);


/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
void USART1_IRQHandler(void) {
    uint8_t input_data;

    if ((USART1->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        /* Receive input data and clear RXNE interrupt flag */
        input_data = USART1->DR;
        /* Send received data back */
        USART1->DR = input_data;
    }
}


/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    /* Init system clock */
    SystemClock_Init();
    /* Init USART1 */
    USART1_Init();

    while (1u) {

    }
}


/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void USART1_Init(void) {
    uint32_t APB2_frequency;
    float mantissa;
    uint32_t fraction;

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   /* enable USART1 clock */
    USART1->CR1 |= USART_CR1_RE;            /* receive enable */
    USART1->CR1 |= USART_CR1_TE;            /* transmit enable */
    USART1->CR1 |= USART_CR1_RXNEIE;        /* receive interrupt enable  */

    /* Set baud rate */
    APB2_frequency = GetPCLK2Frequency();
    mantissa = APB2_frequency / (16.0f * USART1_BAUDRATE);
    fraction = (uint32_t)((mantissa - (uint32_t)mantissa) * 16u);

    USART1->BRR = (USART_BRR_DIV_Mantissa & ((uint32_t)mantissa << 4u)) |
                  (USART_BRR_DIV_Fraction & fraction);

    /* Init pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    /* enable PORTB clock */
    GPIOB->AFR[0] |= (7u << 24u);           /* PB6 in USART1_Tx AF7 */
    GPIOB->AFR[0] |= (7u << 28u);           /* PB7 in USART1_Rx AF7 */
    GPIOB->MODER |= GPIO_MODER_MODER6_1;    /* PB6 in USART1_Tx AF */
    GPIOB->MODER |= GPIO_MODER_MODER7_1;    /* PB7 in USART1_Rx AF */

    /* Set IRQ priority and enable it */
    NVIC_SetPriority(USART1_IRQn, USART1_IRQ_PRIORITY);
    NVIC_EnableIRQ(USART1_IRQn);

    /* Enable USART1 */
    USART1->CR1 |= USART_CR1_UE;            /* enable USART1 */
}
