/* CMSIS */
#include "stm32f4xx.h"

/* User */
#include "RCC.h"


/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    SystemClock_Init();     /* Config SYSCLK for 180 MHz with HSE and PLL */

    while (1u) {

    }
}