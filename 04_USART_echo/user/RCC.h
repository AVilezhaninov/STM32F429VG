#ifndef RCC_H_
#define RCC_H_

/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"
#include "CMSIS\Device\system_stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Exported functions prototypes **********************************************/
/******************************************************************************/
void InitSystemClock(void);

uint32_t GetHCLKFrequency(void);
uint32_t GetPCLK1Frequency(void);
uint32_t GetPCLK2Frequency(void);

void StupidDelay_us(volatile uint32_t delay);
void StupidDelay_ms(volatile uint32_t delay);

#ifdef __cplusplus
}
#endif

#endif /* RCC_H_ */
