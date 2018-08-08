/* CMSIS */
#include "stm32f4xx.h"

/* User */
#include "RCC.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
#define USART1_BAUDRATE       115200u
#define USART1_IRQ_PRIORITY   5u

#define USART2_BAUDRATE       115200u
#define USART2_IRQ_PRIORITY   5u

#define USART3_BAUDRATE       115200u
#define USART3_IRQ_PRIORITY   5u

#define UART4_BAUDRATE        115200u
#define UART4_IRQ_PRIORITY    5u

#define UART5_BAUDRATE        115200u
#define UART5_IRQ_PRIORITY    5u

#define USART6_BAUDRATE       115200u
#define USART6_IRQ_PRIORITY   5u

#define UART7_BAUDRATE        115200u
#define UART7_IRQ_PRIORITY    5u

#define UART8_BAUDRATE        115200u
#define UART8_IRQ_PRIORITY    5u


/******************************************************************************/
/* Static function prototypes *************************************************/
/******************************************************************************/
static void USART1_Init(const uint32_t baudrate, const uint32_t irq_priority);
static void USART2_Init(const uint32_t baudrate, const uint32_t irq_priority);
static void USART3_Init(const uint32_t baudrate, const uint32_t irq_priority);
static void UART4_Init(const uint32_t baudrate, const uint32_t irq_priority);
static void UART5_Init(const uint32_t baudrate, const uint32_t irq_priority);
static void USART6_Init(const uint32_t baudrate, const uint32_t irq_priority);
static void UART7_Init(const uint32_t baudrate, const uint32_t irq_priority);
static void UART8_Init(const uint32_t baudrate, const uint32_t irq_priority);
static void SetUsartBaudRate(USART_TypeDef *const usart,
                             const uint32_t baudrate);


/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
/**
 * USART1 interrupt handler
 * Receive data from USART1 and send it back.
 */
void USART1_IRQHandler(void) {
    uint8_t input_data;

    if ((USART1->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        input_data = USART1->DR;    /* Receive data */
        USART1->DR = input_data;    /* Send it back */
    }
}


/**
 * USART2 interrupt handler
 * Receive data from USART2 and send it back.
 */
void USART2_IRQHandler(void) {
    uint8_t input_data;

    if ((USART2->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        input_data = USART2->DR;    /* Receive data */
        USART2->DR = input_data;    /* Send it back */
    }
}


/**
 * USART3 interrupt handler
 * Receive data from USART3 and send it back.
 */
void USART3_IRQHandler(void) {
    uint8_t input_data;

    if ((USART3->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        input_data = USART3->DR;    /* Receive data */
        USART3->DR = input_data;    /* Send it back */
    }
}


/**
 * UART4 interrupt handler
 * Receive data from UART4 and send it back.
 */
void UART4_IRQHandler(void) {
    uint8_t input_data;

    if ((UART4->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        input_data = UART4->DR;    /* Receive data */
        UART4->DR = input_data;    /* Send it back */
    }
}


/**
 * UART5 interrupt handler
 * Receive data from UART5 and send it back.
 */
void UART5_IRQHandler(void) {
    uint8_t input_data;

    if ((UART5->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        input_data = UART5->DR;    /* Receive data */
        UART5->DR = input_data;    /* Send it back */
    }
}


/**
 * USART6 interrupt handler
 * Receive data from USART6 and send it back.
 */
void USART6_IRQHandler(void) {
    uint8_t input_data;

    if ((USART6->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        input_data = USART6->DR;    /* Receive data */
        USART6->DR = input_data;    /* Send it back */
    }
}


/**
 * UART7 interrupt handler
 * Receive data from UART7 and send it back.
 */
void UART7_IRQHandler(void) {
    uint8_t input_data;

    if ((UART7->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        input_data = UART7->DR;    /* Receive data */
        UART7->DR = input_data;    /* Send it back */
    }
}


/**
 * UART8 interrupt handler
 * Receive data from UART8 and send it back.
 */
void UART8_IRQHandler(void) {
    uint8_t input_data;

    if ((UART8->SR & USART_SR_RXNE) == USART_SR_RXNE) {
        input_data = UART8->DR;    /* Receive data */
        UART8->DR = input_data;    /* Send it back */
    }
}




/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    SystemClock_Init();
    USART1_Init(USART1_BAUDRATE, USART1_IRQ_PRIORITY);
    USART2_Init(USART2_BAUDRATE, USART2_IRQ_PRIORITY);
    USART3_Init(USART3_BAUDRATE, USART3_IRQ_PRIORITY);
    UART4_Init(UART4_BAUDRATE, UART4_IRQ_PRIORITY);
    UART5_Init(UART5_BAUDRATE, UART5_IRQ_PRIORITY);
    USART6_Init(USART6_BAUDRATE, USART6_IRQ_PRIORITY);
    UART7_Init(UART7_BAUDRATE, UART7_IRQ_PRIORITY);
    UART8_Init(UART8_BAUDRATE, UART8_IRQ_PRIORITY);

    while (1u) {

    }
}




/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void USART1_Init(const uint32_t baudrate, const uint32_t irq_priority) {
    /* Init USART1 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    /* Enable PORTB clock */
    GPIOB->AFR[0] |= (7u << 24u);           /* PB6 in USART1_Tx AF7 */
    GPIOB->AFR[0] |= (7u << 28u);           /* PB7 in USART1_Rx AF7 */
    GPIOB->MODER |= GPIO_MODER_MODER6_1;    /* PB6 in USART1_Tx AF */
    GPIOB->MODER |= GPIO_MODER_MODER7_1;    /* PB7 in USART1_Rx AF */

    /* Init USART1 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   /* Enable USART1 clock */
    USART1->CR1 |= USART_CR1_RE;            /* Receive enable */
    USART1->CR1 |= USART_CR1_TE;            /* Transmit enable */
    USART1->CR1 |= USART_CR1_RXNEIE;        /* Receive interrupt enable  */
    SetUsartBaudRate(USART1, baudrate);     /* Set baud rate */

    NVIC_SetPriority(USART1_IRQn, USART1_IRQ_PRIORITY); /* Set IRQ priority */
    NVIC_EnableIRQ(USART1_IRQn);                        /* Enable interrupt */

    USART1->CR1 |= USART_CR1_UE;            /* Enable USART1 */
}


static void USART2_Init(const uint32_t baudrate, const uint32_t irq_priority) {
    /* Init USART2 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* Enable PORTA clock */
    GPIOA->AFR[0] |= (7u << 8u);            /* PA2 in USART2_Tx AF7 */
    GPIOA->AFR[0] |= (7u << 12u);           /* PA3 in USART2_Rx AF7 */
    GPIOA->MODER |= GPIO_MODER_MODER2_1;    /* PA2 in USART2_Tx AF */
    GPIOA->MODER |= GPIO_MODER_MODER3_1;    /* PA3 in USART2_Rx AF */

    /* Init USART2 */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   /* Enable USART2 clock */
    USART2->CR1 |= USART_CR1_RE;            /* Receive enable */
    USART2->CR1 |= USART_CR1_TE;            /* Transmit enable */
    USART2->CR1 |= USART_CR1_RXNEIE;        /* Receive interrupt enable  */
    SetUsartBaudRate(USART2, baudrate);     /* Set baud rate */

    NVIC_SetPriority(USART2_IRQn, USART2_IRQ_PRIORITY); /* Set IRQ priority */
    NVIC_EnableIRQ(USART2_IRQn);                        /* Enable interrupt */

    USART2->CR1 |= USART_CR1_UE;            /* Enable USART2 */
}


static void USART3_Init(const uint32_t baudrate, const uint32_t irq_priority) {
    /* Init USART3 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    /* Enable PORTB clock */
    GPIOB->AFR[1] |= (7u << 8u);            /* PB10 in USART3_Tx AF7 */
    GPIOB->AFR[1] |= (7u << 12u);           /* PB11 in USART3_Rx AF7 */
    GPIOB->MODER |= GPIO_MODER_MODER10_1;   /* PB10 in USART3_Tx AF */
    GPIOB->MODER |= GPIO_MODER_MODER11_1;   /* PB11 in USART3_Rx AF */

    /* Init USART3 */
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;   /* Enable USART3 clock */
    USART3->CR1 |= USART_CR1_RE;            /* Receive enable */
    USART3->CR1 |= USART_CR1_TE;            /* Transmit enable */
    USART3->CR1 |= USART_CR1_RXNEIE;        /* Receive interrupt enable  */
    SetUsartBaudRate(USART3, baudrate);     /* Set baud rate */

    NVIC_SetPriority(USART3_IRQn, USART3_IRQ_PRIORITY); /* Set IRQ priority */
    NVIC_EnableIRQ(USART3_IRQn);                        /* Enable interrupt */

    USART3->CR1 |= USART_CR1_UE;            /* Enable USART3 */
}


static void UART4_Init(const uint32_t baudrate, const uint32_t irq_priority) {
    /* Init UART4 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    /* Enable PORTC clock */
    GPIOC->AFR[1] |= (8u << 8u);            /* PC10 in UART4_Tx AF8 */
    GPIOC->AFR[1] |= (8u << 12u);           /* PC11 in UART4_Rx AF8 */
    GPIOC->MODER |= GPIO_MODER_MODER10_1;   /* PC10 in UART4_Tx AF */
    GPIOC->MODER |= GPIO_MODER_MODER11_1;   /* PC11 in UART4_Rx AF */

    /* Init UART4 */
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;    /* Enable UART4 clock */
    UART4->CR1 |= USART_CR1_RE;             /* Receive enable */
    UART4->CR1 |= USART_CR1_TE;             /* Transmit enable */
    UART4->CR1 |= USART_CR1_RXNEIE;         /* Receive interrupt enable  */
    SetUsartBaudRate(UART4, baudrate);      /* Set baud rate */

    NVIC_SetPriority(UART4_IRQn, UART4_IRQ_PRIORITY); /* Set IRQ priority */
    NVIC_EnableIRQ(UART4_IRQn);                       /* Enable interrupt */

    UART4->CR1 |= USART_CR1_UE;             /* Enable UART4 */
}


static void UART5_Init(const uint32_t baudrate, const uint32_t irq_priority) {
    /* Init UART5 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    /* Enable PORTC clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;    /* Enable PORTD clock */
    GPIOC->AFR[1] |= (8u << 16u);           /* PC12 in UART5_Tx AF8 */
    GPIOD->AFR[0] |= (8u << 8u);            /* PD2 in UART5_Rx AF8 */
    GPIOC->MODER |= GPIO_MODER_MODER12_1;   /* PC12 in UART5_Tx AF */
    GPIOD->MODER |= GPIO_MODER_MODER2_1;    /* PD2 in UART5_Rx AF */

    /* Init UART5 */
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;    /* Enable UART5 clock */
    UART5->CR1 |= USART_CR1_RE;             /* Receive enable */
    UART5->CR1 |= USART_CR1_TE;             /* Transmit enable */
    UART5->CR1 |= USART_CR1_RXNEIE;         /* Receive interrupt enable  */
    SetUsartBaudRate(UART5, baudrate);      /* Set baud rate */

    NVIC_SetPriority(UART5_IRQn, UART5_IRQ_PRIORITY); /* Set IRQ priority */
    NVIC_EnableIRQ(UART5_IRQn);                       /* Enable interrupt */

    UART5->CR1 |= USART_CR1_UE;             /* Enable UART5 */
}


static void USART6_Init(const uint32_t baudrate, const uint32_t irq_priority) {
    /* Init USART6 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    /* Enable PORTC clock */
    GPIOC->AFR[0] |= (8u << 24u);           /* PC6 in USART6_Tx AF8 */
    GPIOC->AFR[0] |= (8u << 28u);           /* PC7 in USART6_Rx AF8 */
    GPIOC->MODER |= GPIO_MODER_MODER6_1;    /* PC6 in USART6_Tx AF */
    GPIOC->MODER |= GPIO_MODER_MODER7_1;    /* PC7 in USART6_Rx AF */

    /* Init USART6 */
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;   /* Enable USART6 clock */
    USART6->CR1 |= USART_CR1_RE;            /* Receive enable */
    USART6->CR1 |= USART_CR1_TE;            /* Transmit enable */
    USART6->CR1 |= USART_CR1_RXNEIE;        /* Receive interrupt enable  */
    SetUsartBaudRate(USART6, baudrate);     /* Set baud rate */

    NVIC_SetPriority(USART6_IRQn, USART6_IRQ_PRIORITY); /* Set IRQ priority */
    NVIC_EnableIRQ(USART6_IRQn);                        /* Enable interrupt */

    USART6->CR1 |= USART_CR1_UE;            /* Enable USART6 */
}


static void UART7_Init(const uint32_t baudrate, const uint32_t irq_priority) {
    /* Init UART7 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;    /* Enable PORTE clock */
    GPIOE->AFR[0] |= (8u << 28u);           /* PE7 in UART7_Rx AF8 */
    GPIOE->AFR[1] |= (8u << 0u);            /* PE8 in UART7_Tx AF8 */
    GPIOE->MODER |= GPIO_MODER_MODER7_1;    /* PE7 in UART7_Rx AF */
    GPIOE->MODER |= GPIO_MODER_MODER8_1;    /* PE8 in UART7_Tx AF */

    /* Init UART7 */
    RCC->APB1ENR |= RCC_APB1ENR_UART7EN;    /* Enable UART7 clock */
    UART7->CR1 |= USART_CR1_RE;             /* Receive enable */
    UART7->CR1 |= USART_CR1_TE;             /* Transmit enable */
    UART7->CR1 |= USART_CR1_RXNEIE;         /* Receive interrupt enable  */
    SetUsartBaudRate(UART7, baudrate);      /* Set baud rate */

    NVIC_SetPriority(UART7_IRQn, UART7_IRQ_PRIORITY); /* Set IRQ priority */
    NVIC_EnableIRQ(UART7_IRQn);                       /* Enable interrupt */

    UART7->CR1 |= USART_CR1_UE;             /* Enable UART7 */
}


static void UART8_Init(const uint32_t baudrate, const uint32_t irq_priority) {
    /* Init UART8 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;    /* Enable PORTE clock */
    GPIOE->AFR[0] |= (8u << 0u);            /* PE0 in UART8_Rx AF8 */
    GPIOE->AFR[0] |= (8u << 4u);            /* PE1 in UART8_Tx AF8 */
    GPIOE->MODER |= GPIO_MODER_MODER0_1;    /* PE0 in UART8_Rx AF */
    GPIOE->MODER |= GPIO_MODER_MODER1_1;    /* PE1 in UART8_Tx AF */

    /* Init UART8 */
    RCC->APB1ENR |= RCC_APB1ENR_UART8EN;    /* Enable UART8 clock */
    UART8->CR1 |= USART_CR1_RE;             /* Receive enable */
    UART8->CR1 |= USART_CR1_TE;             /* Transmit enable */
    UART8->CR1 |= USART_CR1_RXNEIE;         /* Receive interrupt enable  */
    SetUsartBaudRate(UART8, baudrate);      /* Set baud rate */

    NVIC_SetPriority(UART8_IRQn, UART8_IRQ_PRIORITY); /* Set IRQ priority */
    NVIC_EnableIRQ(UART8_IRQn);                       /* Enable interrupt */

    UART8->CR1 |= USART_CR1_UE;             /* Enable UART8 */
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
