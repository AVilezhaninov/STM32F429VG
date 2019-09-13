#ifndef CAN_H_
#define CAN_H_

/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/* Exported definitions *******************************************************/
/******************************************************************************/
#define CAN_MAX_DATA_NUM  8u

/******************************************************************************/
/* Exported constants *********************************************************/
/******************************************************************************/
enum { CAN_DATA_FRAME = 0u, CAN_REMOTE_FRAME = 2u };

/******************************************************************************/
/* Exported type definitions **************************************************/
/******************************************************************************/
typedef struct {
  uint32_t ExtId;
  uint8_t RTR;
  uint8_t DLC;
  uint8_t Data[CAN_MAX_DATA_NUM];
} CAN_Tx_t;

typedef struct {
  uint32_t ExtId;
  uint8_t RTR;
  uint8_t DLC;
  uint8_t Data[CAN_MAX_DATA_NUM];
  uint8_t FMI;
} CAN_Rx_t;

/******************************************************************************/
/* Exported functions prototypes **********************************************/
/******************************************************************************/
void CAN_Init();
bool CAN_Transmit(const CAN_Tx_t *const tx_message);
bool CAN_Receive(CAN_Rx_t *const rx_message);
void CAN_EnableClock();
void CAN_DisableClock();

#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */
