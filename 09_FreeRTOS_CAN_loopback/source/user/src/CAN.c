#include "CAN.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
#define CAN_IRQ_PRIORITY    5u


/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void FilterInit(void);


/******************************************************************************/
/* Exported functions *********************************************************/
/******************************************************************************/
void CAN_Init(void) {
    /* Init pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* enable PORTA clock */
    GPIOA->AFR[1u] |= (9u << 12u);          /* PA11 in CAN1_RX AF9 */
    GPIOA->AFR[1u] |= (9u << 16u);          /* PA12 in CAN1_TX AF9 */
    GPIOA->MODER |= GPIO_MODER_MODER11_1;   /* PA11 in CAN1_RX AF */
    GPIOA->MODER |= GPIO_MODER_MODER12_1;   /* PA12 in CAN1_TX AF */

    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;     /* enable CAN clock */

    /* Request initialization */
    CAN1->MCR |= CAN_MCR_INRQ ;
    while((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) {
        ;
    }

    /* Turn on bus-off management */
    CAN1->MCR |= CAN_MCR_ABOM;
    /* Set baud rate to 1 MBit/s */
    CAN1->BTR = (2u << CAN_BTR_TS2_Pos) | (4u << CAN_BTR_TS1_Pos) | 4u;
    /* Enable FIFO 0 message pending interrup */
    CAN1->IER |= CAN_IER_FMPIE0;


    /*------------------------------------------------------------------------*/
    /* Debug loopback mode ---------------------------------------------------*/
    /*------------------------------------------------------------------------*/
    /* Enable loop back mode */
    CAN1->BTR |= CAN_BTR_LBKM;
    /* Enbale silent mode */
    CAN1->BTR |= CAN_BTR_SILM;


    /* Request leave initialization */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {
        ;
    }

    /* Init CAN filters */
    FilterInit();

    /* Exit from sleep mode */
    CAN1->MCR &= ~CAN_MCR_SLEEP;

    /* Enable iterrupts */
    NVIC_SetPriority(CAN1_RX0_IRQn, CAN_IRQ_PRIORITY);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
}


/**
 * Receive CAN message from mailbox.
 * @param  CAN_RX_message Pointer to CAN message
 * @return                True if message received
 */
bool CAN_Receive(CAN_RX_message_t *const CAN_RX_message) {
    bool message_received = false;
    int8_t mailbox_num;

    if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0u) {
        message_received = true;
        mailbox_num = (CAN1->RF0R & CAN_RF0R_FMP0) - 1u;
        CAN_RX_message->RIR = CAN1->sFIFOMailBox[mailbox_num].RIR;
        CAN_RX_message->RDTR = CAN1->sFIFOMailBox[mailbox_num].RDTR;
        CAN_RX_message->RDLR = CAN1->sFIFOMailBox[mailbox_num].RDLR;
        CAN_RX_message->RDHR = CAN1->sFIFOMailBox[mailbox_num].RDHR;

        /* Clear mailbox flag */
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
    return message_received;
}


/**
 * Transmit CAN message to mailbox.
 * @param  CAN_TX_message Pointer to CAN message
 * @return                True if message placed in mailbox
 */
bool CAN_Transmit(const CAN_TX_message_t *const CAN_TX_message) {
    bool message_transmitted = false;
    uint8_t first_empty_mailbox = 0xFFu;

    /* Define first empty mailbox */
    if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
        first_empty_mailbox = 0u;
    } else if ((CAN1->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
        first_empty_mailbox = 1u;
    } else if ((CAN1->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) {
        first_empty_mailbox = 2u;
    } else {
    }

    if (first_empty_mailbox != 0xFFu) {
        message_transmitted = true;
        CAN1->sTxMailBox[first_empty_mailbox].TIR = CAN_TX_message->TIR;
        CAN1->sTxMailBox[first_empty_mailbox].TDTR = CAN_TX_message->TDTR;
        CAN1->sTxMailBox[first_empty_mailbox].TDLR = CAN_TX_message->TDLR;
        CAN1->sTxMailBox[first_empty_mailbox].TDHR = CAN_TX_message->TDHR;
        CAN1->sTxMailBox[first_empty_mailbox].TIR |= CAN_TI0R_TXRQ;
    }

    return message_transmitted;
}


/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
/**
 * Init CAN filters for receiving all messages from bus
 */
static void FilterInit(void) {
    /* Initialization mode for the filter */
    CAN1->FMR |= CAN_FMR_FINIT;

    /* Filter 0 */
    CAN1->FS1R |= CAN_FS1R_FSC0;
    CAN1->sFilterRegister[0u].FR1 = 0x00000000u;
    CAN1->sFilterRegister[0u].FR2 = 0x00000000u;
    CAN1->FA1R |= CAN_FA1R_FACT0;

    /* Leave the initialization mode for the filter */
    CAN1->FMR &= ~CAN_FMR_FINIT;
}
