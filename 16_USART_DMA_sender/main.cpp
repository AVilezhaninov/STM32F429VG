/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/* User */
#include "user\RCC.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define USART1_BAUDRATE       115200u
#define USART1_IRQ_PRIORITY   5u
#define ECHO_STRING           "Echo string"

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitUsart1(const uint32_t baudrate, const uint32_t irq_priority);
static void SetUsartBaudRate(USART_TypeDef *const usart,
                             const uint32_t baudrate);

/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
extern "C" {

/**
 * USART1 interrupt handler
 * Receive data from USART1 and send it back.
 */
void USART1_IRQHandler() {
  if ((USART1->SR & USART_SR_RXNE) == USART_SR_RXNE) {
    (void)USART1->DR;                 /* Receive data */
    DMA2->HIFCR |= DMA_HIFCR_CTCIF7;  /* Clear interrupt flag */
    DMA2_Stream7->CR |= DMA_SxCR_EN;  /* Start DMA transmittion */
  }
}

} /* extern "C" */

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main() {
  InitSystemClock();
  InitUsart1(USART1_BAUDRATE, USART1_IRQ_PRIORITY);

  while (1) {
    ;
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void InitUsart1(const uint32_t baudrate, const uint32_t irq_priority) {
  /* Init USART1 pins */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* Enable PORTB clock */
  GPIOB->AFR[0] |= (7u << 24u);        /* PB6 in USART1_Tx AF7 */
  GPIOB->AFR[0] |= (7u << 28u);        /* PB7 in USART1_Rx AF7 */
  GPIOB->MODER |= GPIO_MODER_MODER6_1; /* PB6 in USART1_Tx AF */
  GPIOB->MODER |= GPIO_MODER_MODER7_1; /* PB7 in USART1_Rx AF */

  /* Init USART1 */
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; /* Enable USART1 clock */
  USART1->CR1 |= USART_CR1_RE;          /* Receive enable */
  USART1->CR1 |= USART_CR1_TE;          /* Transmit enable */
  USART1->CR3 |= USART_CR3_DMAT;        /* Enable DMA transmission */
  USART1->CR1 |= USART_CR1_RXNEIE;      /* Receive interrupt enable  */
  SetUsartBaudRate(USART1, baudrate);   /* Set baud rate */

  NVIC_SetPriority(USART1_IRQn, USART1_IRQ_PRIORITY); /* Set IRQ priority */
  NVIC_EnableIRQ(USART1_IRQn);                        /* Enable interrupt */

  /* Init DMA */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;         /* Enable DMA2 clock */
  DMA2_Stream7->CR |= DMA_SxCR_CHSEL_2;       /* Channel 4 */
  DMA2_Stream7->CR |= DMA_SxCR_MINC;          /* Memory increment */
  DMA2_Stream7->CR |= DMA_SxCR_DIR_0;         /* Memory to peripheral */
  DMA2_Stream7->NDTR = 11u;                   /* Data length */
  DMA2_Stream7->PAR = (uint32_t)&USART1->DR;  /* Peripheral address */
  DMA2_Stream7->M0AR = (uint32_t)ECHO_STRING; /* Memory address */

  USART1->CR1 |= USART_CR1_UE;  /* Enable USART1 */
}

/**
 * Set USART baud rate
 * @param usart    USART instance
 * @param baudrate Required baud rate
 */
static void SetUsartBaudRate(USART_TypeDef *const usart,
                             const uint32_t baudrate) {
  uint32_t bus_clock;
  float mantissa;
  uint32_t fraction;

  /* Get bus clock for current USART */
  if ((usart == USART1) || (usart == USART6)) {
    bus_clock = GetPCLK2Frequency();
  } else {
    bus_clock = GetPCLK1Frequency();
  }

  /* Set baud rate */
  mantissa = bus_clock / (16.0f * baudrate);
  fraction = (uint32_t)((mantissa - (uint32_t)mantissa) * 16u);
  usart->BRR = (USART_BRR_DIV_Mantissa & ((uint32_t)mantissa << 4u)) |
               (USART_BRR_DIV_Fraction & fraction);
}
