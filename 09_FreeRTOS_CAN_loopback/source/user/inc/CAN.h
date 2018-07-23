#ifndef CAN_H
#define CAN_H


#include <stdbool.h>
#include "stm32f4xx.h"


/******************************************************************************/
/* Constants ******************************************************************/
/******************************************************************************/
enum {
    CAN_DATA_FRAME,
    CAN_REMOTE_FRAME
};


/******************************************************************************/
/* Type definition ************************************************************/
/******************************************************************************/
typedef struct {
    uint32_t RIR;
    uint32_t RDTR;
    uint32_t RDLR;
    uint32_t RDHR;
} CAN_RX_message_t ;


typedef struct {
    uint32_t TIR;
    uint32_t TDTR;
    uint32_t TDLR;
    uint32_t TDHR;
} CAN_TX_message_t;


/******************************************************************************/
/* Exported functions *********************************************************/
/******************************************************************************/
void CAN_Init(void);
bool CAN_Receive(CAN_RX_message_t *const CAN_RX_message);
bool CAN_Transmit(const CAN_TX_message_t *const CAN_TX_message);


#endif /* CAN_H */
