/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/* User */
#include "user\RCC.h"

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitSpi1();
static void SetSpi1Cs();
static void ResetSpi1Cs();

static void InitSpi2();
static void SetSpi2Cs();
static void ResetSpi2Cs();

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
  uint8_t spi_data = 0u;

  InitSystemClock();
  InitSpi1();
  InitSpi2();

  while (1u) {
    /* SPI1 loopback */
    ResetSpi1Cs();
    SPI1->DR = spi_data;
    while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {
      ;
    }
    spi_data = SPI1->DR;
    SetSpi1Cs();

    /* SPI2 loopback */
    ResetSpi2Cs();
    SPI2->DR = spi_data;
    while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {
      ;
    }
    spi_data = SPI2->DR;
    SetSpi2Cs();

    ++spi_data;
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void InitSpi1() {
  /* Init SPI1 */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;   /* Enable SPI clock */
  SPI1->CR1 |= SPI_CR1_BR_0;            /* SPI clock = Fpclk/4 */
  SPI1->CR1 |= SPI_CR1_MSTR;            /* SPI in master mode */
  SPI1->CR1 &= ~SPI_CR1_CPOL;           /* CPOL = 0 */
  SPI1->CR1 &= ~SPI_CR1_CPHA;           /* CPHA = 0 */
  SPI1->CR1 |= SPI_CR1_SSM;             /* SSM = 1 */
  SPI1->CR1 |= SPI_CR1_SSI;             /* SSI = 1 */
  SPI1->CR2 |= SPI_CR2_RXNEIE;          /* Rx interrupt enable */
  SPI1->CR1 |= SPI_CR1_SPE;             /* Enable SPI */

  /* Init SPI1 GPIO */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   /* Enable GPIOB clock */
  GPIOB->AFR[0] |= (5u << 12u);          /* PB3 in SPI1_SCK AF5 */
  GPIOB->AFR[0] |= (5u << 16u);          /* PB4 in SPI1_MISO AF5 */
  GPIOB->AFR[0] |= (5u << 20u);          /* PB5 in SPI1_MOSI AF5 */
  GPIOB->MODER |= GPIO_MODER_MODER3_1;   /* PB3 in SPI1_SCK AF */
  GPIOB->MODER |= GPIO_MODER_MODER4_1;   /* PB4 in SPI1_MISO AF */
  GPIOB->MODER |= GPIO_MODER_MODER5_1;   /* PB5 in SPI1_MOSI AF */

  GPIOB->MODER |= GPIO_MODER_MODER6_0;   /* PB6 as ~CS */
  SetSpi1Cs();                           /* Set chip select high */
}

static void SetSpi1Cs() {
  GPIOB->BSRR |= GPIO_BSRR_BS_6;
}

static void ResetSpi1Cs() {
  GPIOB->BSRR |= GPIO_BSRR_BR_6;
}

static void InitSpi2() {
  /* Init SPI1 */
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;   /* Enable SPI clock */
  SPI2->CR1 |= SPI_CR1_BR_0;            /* SPI clock = Fpclk/4 */
  SPI2->CR1 |= SPI_CR1_MSTR;            /* SPI in master mode */
  SPI2->CR1 &= ~SPI_CR1_CPOL;           /* CPOL = 0 */
  SPI2->CR1 &= ~SPI_CR1_CPHA;           /* CPHA = 0 */
  SPI2->CR1 |= SPI_CR1_SSM;             /* SSM = 1 */
  SPI2->CR1 |= SPI_CR1_SSI;             /* SSI = 1 */
  SPI2->CR2 |= SPI_CR2_RXNEIE;          /* Rx interrupt enable */
  SPI2->CR1 |= SPI_CR1_SPE;             /* Enable SPI */

  /* Init SPI1 GPIO */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   /* Enable GPIOB clock */
  GPIOB->AFR[1] |= (5u << 20u);          /* PB13 in SPI1_SCK AF5 */
  GPIOB->AFR[1] |= (5u << 24u);          /* PB14 in SPI1_MISO AF5 */
  GPIOB->AFR[1] |= (5u << 28u);          /* PB15 in SPI1_MOSI AF5 */
  GPIOB->MODER |= GPIO_MODER_MODER13_1;  /* PB13 in SPI1_SCK AF */
  GPIOB->MODER |= GPIO_MODER_MODER14_1;  /* PB14 in SPI1_MISO AF */
  GPIOB->MODER |= GPIO_MODER_MODER15_1;  /* PB15 in SPI1_MOSI AF */

  GPIOB->MODER |= GPIO_MODER_MODER11_0;  /* PB11 as ~CS */
  SetSpi2Cs();                           /* Set chip select high */
}

static void SetSpi2Cs() {
  GPIOB->BSRR |= GPIO_BSRR_BS_11;
}

static void ResetSpi2Cs() {
  GPIOB->BSRR |= GPIO_BSRR_BR_11;
}
