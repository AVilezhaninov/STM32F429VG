#include "CAN.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define UNDEFINED_MAILBOX   0xFFu
#define CAN_IRQ_PRIORITY    5u

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitGPIO();
static void EnableCAN();
static void EnterInitMode();
static void LeaveInitMode();
static void InitCANRegisters();
static void InitFilters();
static void EnableInterrupts();

/******************************************************************************/
/* Exported functions *********************************************************/
/******************************************************************************/
void CAN_Init() {
  InitGPIO();
  EnableCAN();
  EnterInitMode();
  InitCANRegisters();
  LeaveInitMode();
  InitFilters();
  EnableInterrupts();
}

/**
 * Try to put CAN message to mailbox.
 * @param  message Pointer to CAN message
 * @return         True if message was put in mailbox
 */
bool CAN_Transmit(const CAN_Tx_t *const message) {
  bool message_in_mailbox = false;
  uint8_t mailbox = UNDEFINED_MAILBOX;

  /* Looking for first empty mailbox */
  if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    mailbox = 0u;
  } else if ((CAN1->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
    mailbox = 1u;
  } else if ((CAN1->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) {
    mailbox = 2u;
  } else {
  }

  /* Fill tx mailbox */
  if (mailbox != UNDEFINED_MAILBOX) {
    message_in_mailbox = true;

    CAN1->sTxMailBox[mailbox].TIR = message->ExtId << CAN_TI0R_EXID_Pos;
    CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_IDE;
    CAN1->sTxMailBox[mailbox].TIR |= message->RTR;

    CAN1->sTxMailBox[mailbox].TDTR = CAN_TDT0R_DLC;
    CAN1->sTxMailBox[mailbox].TDTR &= message->DLC;

    CAN1->sTxMailBox[mailbox].TDLR = message->Data[0u];
    CAN1->sTxMailBox[mailbox].TDLR |= message->Data[1u] << 8u;
    CAN1->sTxMailBox[mailbox].TDLR |= message->Data[2u] << 16u;
    CAN1->sTxMailBox[mailbox].TDLR |= message->Data[3u] << 24u;

    CAN1->sTxMailBox[mailbox].TDHR = message->Data[4u];
    CAN1->sTxMailBox[mailbox].TDHR |= message->Data[5u] << 8u;
    CAN1->sTxMailBox[mailbox].TDHR |= message->Data[6u] << 16u;
    CAN1->sTxMailBox[mailbox].TDHR |= message->Data[7u] << 24u;

    CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;
  }

  return message_in_mailbox;
}

/**
 * Try to receive CAN message from mailbox.
 * @param  message Pointer to CAN message
 * @return         True if message received
 */
bool CAN_Receive(CAN_Rx_t *const message) {
  bool message_received = false;
  int8_t mailbox;

  if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0u) {
    message_received = true;
    mailbox = (CAN1->RF0R & CAN_RF0R_FMP0) - 1u;

    message->ExtId = CAN1->sFIFOMailBox[mailbox].RIR >> CAN_RI0R_EXID_Pos;
    message->RTR = CAN1->sFIFOMailBox[mailbox].RIR & CAN_RI0R_RTR;
    message->DLC = CAN1->sFIFOMailBox[mailbox].RDTR & CAN_RDT0R_DLC;
    message->FMI = CAN1->sFIFOMailBox[mailbox].RDTR >> CAN_RDT0R_FMI_Pos;

    message->Data[0u] = CAN1->sFIFOMailBox[mailbox].RDLR & 0xFFu;
    message->Data[1u] = (CAN1->sFIFOMailBox[mailbox].RDLR >> 8u) & 0xFFu;
    message->Data[2u] = (CAN1->sFIFOMailBox[mailbox].RDLR >> 16u) & 0xFFu;
    message->Data[3u] = (CAN1->sFIFOMailBox[mailbox].RDLR >> 24u) & 0xFFu;

    message->Data[4u] = CAN1->sFIFOMailBox[mailbox].RDHR & 0xFFu;
    message->Data[5u] = (CAN1->sFIFOMailBox[mailbox].RDHR >> 8u) & 0xFFu;
    message->Data[6u] = (CAN1->sFIFOMailBox[mailbox].RDHR >> 16u) & 0xFFu;
    message->Data[7u] = (CAN1->sFIFOMailBox[mailbox].RDHR >> 24u) & 0xFFu;

    CAN1->RF0R |= CAN_RF0R_RFOM0; /* clear mailbox flag */
  }

  return message_received;
}

void CAN_EnableClock() {
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
}

void CAN_DisableClock() {
  RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
}

/******************************************************************************/
/* Private function ***********************************************************/
/******************************************************************************/
static void InitGPIO() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* enable PORT clock */
  GPIOA->AFR[1u] |= (9u << 12u);          /* PA11 in CAN1_RX AF9 */
  GPIOA->AFR[1u] |= (9u << 16u);          /* PA12 in CAN1_TX AF9 */
  GPIOA->MODER |= GPIO_MODER_MODER11_1;   /* PA11 in CAN1_RX AF */
  GPIOA->MODER |= GPIO_MODER_MODER12_1;   /* PA12 in CAN1_TX AF */
}

static void EnableCAN() {
  CAN_EnableClock();
  CAN1->MCR &= ~CAN_MCR_SLEEP;            /* exit from sleep mode */
}

static void EnterInitMode() {
  /* Request init mode */
  CAN1->MCR |= CAN_MCR_INRQ;
  /* Wait init mode */
  while ((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) {
    ;
  }
}

static void LeaveInitMode() {
  /* Request normal mode */
  CAN1->MCR &= ~CAN_MCR_INRQ;
  /* Wait normal mode */
  while ((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {
    ;
  }
}

static void InitCANRegisters() {
  CAN1->MCR |= CAN_MCR_ABOM;    /* Set automatic bus-off management */
  CAN1->BTR = 0x001B0002;       /* Value from baudrate calculator */
  CAN1->IER |= CAN_IER_FMPIE0;  /* FIFO message pending interrupt enable */

  /*------------------------------------------------------------------------*/
  /* Debug loopback mode ---------------------------------------------------*/
  /*------------------------------------------------------------------------*/
  /* Enable loop back mode */
  CAN1->BTR |= CAN_BTR_LBKM;
  /* Enbale silent mode */
  CAN1->BTR |= CAN_BTR_SILM;
}

static void InitFilters() {
  CAN1->FMR |= CAN_FMR_FINIT;   /* Initialization mode for the filter */

  /* Filter 0 */
  CAN1->FS1R |= CAN_FS1R_FSC0;  /* Single 32-bit scale configuration */
  CAN1->sFilterRegister[0u].FR1 = 0x0000u;
  CAN1->sFilterRegister[0u].FR2 = 0x0000u;
  CAN1->FA1R |= CAN_FA1R_FACT0; /* Active filter */

  CAN1->FMR &= ~CAN_FMR_FINIT;  /* Active filters mode */
}

static void EnableInterrupts() {
  NVIC_SetPriority(CAN1_RX0_IRQn, CAN_IRQ_PRIORITY);
  NVIC_EnableIRQ(CAN1_RX0_IRQn);
}
