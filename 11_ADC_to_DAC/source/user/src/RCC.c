#include "RCC.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M       8u
#define PLL_N       360u

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P       2u

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q       7u

#define AHB_PRESCALER           RCC_CFGR_HPRE_DIV1
#define APB1_PRESCALER          RCC_CFGR_PPRE1_DIV4
#define APB2_PRESCALER          RCC_CFGR_PPRE2_DIV2

/* Timeout for HSE start up */
#define HSE_STARTUP_TIMEOUT     0x05000u


/******************************************************************************/
/* Exported functions *********************************************************/
/******************************************************************************/

/* System clocks init ------------------------------------------------------- */
/**
* @brief  Configures the System clock source, PLL Multiplier and Divider factors
*         AHB/APBx prescalers and Flash settings
* @Note   This function should be called only once the RCC clock configuration
*         is reset to the default reset state (done in SystemInit() function).
* @param  None
* @retval None
*/
void SystemClock_Init(void) {
    /**************************************************************************/
    /*            PLL (clocked by HSE) used as System clock source            */
    /**************************************************************************/
    uint32_t StartUpCounter = 0u;
    uint32_t HSEStatus = 0u;

    /* Enable HSE */
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    while ((HSEStatus == 0u) && (StartUpCounter < HSE_STARTUP_TIMEOUT)) {
        HSEStatus = RCC->CR & RCC_CR_HSERDY;
        ++StartUpCounter;
    }

    if (HSEStatus == RCC_CR_HSERDY) {
        /* Voltage regulator Scale 1 */
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_VOS;

        /* HCLK = SYSCLK / 1*/
        RCC->CFGR |= AHB_PRESCALER;

        /* PCLK2 = HCLK / 2*/
        RCC->CFGR |= APB2_PRESCALER;

        /* PCLK1 = HCLK / 4*/
        RCC->CFGR |= APB1_PRESCALER;

        /* Configure the main PLL */
        RCC->PLLCFGR = PLL_M | (PLL_N << 6u) | (((PLL_P >> 1u) - 1u) << 16u)
                     | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24u);

        /* Enable the main PLL */
        RCC->CR |= RCC_CR_PLLON;

        /* Wait till the main PLL is ready */
        while ((RCC->CR & RCC_CR_PLLRDY) == 0u) {}

#if defined (STM32F427xx) || defined (STM32F429xx) || defined (STM32F439xx) || defined (STM32F446xx)
        /* Enable the Over-drive to extend the clock frequency to 180 Mhz */
        PWR->CR |= PWR_CR_ODEN;
        while ((PWR->CSR & PWR_CSR_ODRDY) == 0u) {}

        PWR->CR |= PWR_CR_ODSWEN;
        while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0u) {}

        /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
#endif /* STM32F427xx || STM32F429xx  */

#if defined (STM32F407xx)
        /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
#endif /* STM32F407xx  */

#if defined (STM32F401xx)
        /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_2WS;
#endif /* STM32F401xx */

        /* Select the main PLL as system clock source */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= RCC_CFGR_SW_PLL;

        /* Wait till the main PLL is used as system clock source */
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL) {}

        SystemCoreClockUpdate();
    } else {
        /* If HSE fails to start-up, the application will have wrong clock
        configuration. User can add here some code to deal with this error */
        while (1) {}
    }
}


/**
 * Get HCLK frequency
 * @return  HCLK frequency
 */
uint32_t GetHCLKFrequency(void) {
    return SystemCoreClock;
}


/**
 * Get PCLK1 frequency
 * @return  PCLK1 frequency
 */
uint32_t GetPCLK1Frequency(void) {
    return GetHCLKFrequency() >>
           APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
}


/**
 * Get PCLK2 frequency
 * @return  PCLK2 frequency
 */
uint32_t GetPCLK2Frequency(void) {
    return GetHCLKFrequency() >>
           APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
}


/**
 * Blocking delay in us
 * @param delay [us]
 */
void StupidDelay_us(volatile uint32_t delay) {
    delay *= (SystemCoreClock / 1000000u);

    while (delay-- > 0u) {
        ;
    }
}


/**
 * Blocking delay in ms
 * @param delay [ms]
 */
void StupidDelay_ms(volatile uint32_t delay) {
    delay *= (SystemCoreClock / 1000u);

    while (delay-- > 0u) {
        ;
    }
}
