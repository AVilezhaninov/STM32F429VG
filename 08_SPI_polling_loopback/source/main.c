/* CMSIS */
#include "stm32f4xx.h"

/* User */
#include "RCC.h"


/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void CSSet(void);
static void CSReset(void);


/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    /* Init system clock */
    SystemClock_Init();

    /* Init SPI1 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;     /* Enable SPI1 clock */
    SPI1->CR1 |= SPI_CR1_BR_0;              /* SPI clock = Fpclk/4 */
    SPI1->CR1 |= SPI_CR1_MSTR;              /* SPI in master mode */
    SPI1->CR1 &= ~SPI_CR1_CPOL;             /* CPOL = 0 */
    SPI1->CR1 &= ~SPI_CR1_CPHA;             /* CPHA = 0 */
    SPI1->CR1 |= SPI_CR1_SSM;               /* SSM = 1 */
    SPI1->CR1 |= SPI_CR1_SSI;               /* SSI = 1 */
    SPI1->CR2 |= SPI_CR2_RXNEIE;            /* Rx interrupt enable */
    SPI1->CR1 |= SPI_CR1_SPE;               /* Enable SPI */

    /* Init SPI1 GPIO */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    /* Enable GPIOB clock */
    GPIOB->AFR[0] |= (5u << 12u);           /* PB3 in SPI1_SCK AF5 */
    GPIOB->AFR[0] |= (5u << 16u);           /* PB4 in SPI1_MISO AF5 */
    GPIOB->AFR[0] |= (5u << 20u);           /* PB5 in SPI1_MOSI AF5 */
    GPIOB->MODER |= GPIO_MODER_MODER3_1;    /* PB3 in SPI1_SCK AF */
    GPIOB->MODER |= GPIO_MODER_MODER4_1;    /* PB4 in SPI1_MISO AF */
    GPIOB->MODER |= GPIO_MODER_MODER5_1;    /* PB5 in SPI1_MOSI AF */

    GPIOB->MODER |= GPIO_MODER_MODER6_0;    /* PB6 as ~CS */
    CSSet();                                /* Set chip select high */

    /* Data to send through SPI */
    uint8_t spi_data = 0u;

    while (1u) {
        /* Set chip select low */
        CSReset();

        /* Send data through SPI */
        SPI1->DR = spi_data;
        /* Wait till data come back */
        while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {}
        /* Read received data */
        spi_data = SPI1->DR;

        /* Set chip select high */
        CSSet();

        ++spi_data;
    }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void CSSet(void) {
    GPIOB->BSRR |= GPIO_BSRR_BS_6;
}


static void CSReset(void) {
    GPIOB->BSRR |= GPIO_BSRR_BR_6;
}
