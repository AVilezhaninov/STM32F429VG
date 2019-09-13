/* STD lib */
#include <string>

/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* User */
#include "user\CAN.h"
#include "user\RCC.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define RECEIVE_QUEUE_SIZE    10u
#define TRANSMIT_QUEUE_SIZE   10u
#define TRANSMISSION_DELAY    250u
#define RETRANSMISSION_DELAY  1u
#define MESSAGE_DATA_LENGTH   8u

/******************************************************************************/
/* Private variables **********************************************************/
/******************************************************************************/
static volatile xQueueHandle CanRxQueue;
static volatile xQueueHandle CanTxQueue;

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitAll(void);
static void InitHardware(void);
static void InitLeds(void);
static void ToggleGreenLed(void);
static void ToggleRedLed(void);
static void ToggleBlueLed(void);
static void InitRTOSObjects(void);

/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
extern "C" {

/**
 * CAN receive IRQ handler.
 * Receive messages from CAN and send them through queue to CAN receive task.
 */
void CAN1_RX0_IRQHandler(void) {
  portBASE_TYPE higher_priority_task_woken;
  CAN_Rx_t can_rx;  /* Received CAN message */

  /* Receive CAN message from all CAN RX mailboxes */
  while (CAN_Receive(&can_rx)) {
    /* Send message to CAN receive task */
    higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(CanRxQueue, &can_rx, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
      taskYIELD();
    }
  }
}

} /* extern "C" */

/******************************************************************************/
/* Tasks **********************************************************************/
/******************************************************************************/
/**
 * CAN receive task.
 * Receive CAN messages from IRQ handler.
 * @param pvParameters None
 */
void CANReceiveTask(void *pvParameters) {
  (void)pvParameters;
  CAN_Rx_t can_rx; /* Received CAN message */

  while (1u) {
    /* Receive CAN message from CAN IRQ */
    xQueueReceive(CanRxQueue, &can_rx, portMAX_DELAY);
    /* Indicate message receiving from CAN */
    ToggleGreenLed();

    /* Handle received message here */
  }
}

/**
 * CAN transmit task.
 * Receive CAN message from other tasks and try to send it through CAN.
 * @param pvParameters None
 */
void CANTransmitTask(void *pvParameters) {
  (void)pvParameters;
  CAN_Tx_t can_tx; /* CAN message to send */

  while (1u) {
    /* Receive CAN message from queue */
    xQueueReceive(CanTxQueue, &can_tx, portMAX_DELAY);
    /* Indicate message receiving from queue */
    ToggleBlueLed();

    /* Try to put CAN message in mailbox */
    while (!CAN_Transmit(&can_tx)) {
      /* CAN retransmission delay if mailboxes are busy */
      vTaskDelay(RETRANSMISSION_DELAY);
    }
  }
}

/**
 * CAN message sender task.
 * Periodically creates CAN message and send them through queue to
 * CAN transmit task.
 * @param pvParameters None
 */
void MessageSenderTask(void *pvParameters) {
  (void)pvParameters;
  CAN_Tx_t can_tx;    /* CAN message to send */
  uint8_t data = 0u;  /* Value to send */

  while (1u) {
    /* Fill CAN message */
    memset(&can_tx, 0u, sizeof(can_tx));
    can_tx.DLC |= MESSAGE_DATA_LENGTH;
    can_tx.Data[0] |= data++;

    /* Send CAN message through queue to CAN transmit task */
    xQueueSend(CanTxQueue, &can_tx, 0u);
    /* Indicate message sending */
    ToggleRedLed();

    /* Delay to next message */
    vTaskDelay(TRANSMISSION_DELAY);
  }
}

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
  InitAll();
  vTaskStartScheduler();

  while (1) {
    ;
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void InitAll(void) {
  InitHardware();
  InitRTOSObjects();
}

static void InitHardware(void) {
  InitSystemClock();
  InitLeds();
  CAN_Init();
}

static void InitLeds(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  /* Enable GPIO clock */
  GPIOD->MODER |= GPIO_MODER_MODER12_0; /* GPIOD pin 12 in push-pull mode */
  GPIOD->MODER |= GPIO_MODER_MODER14_0; /* GPIOD pin 14 in push-pull mode */
  GPIOD->MODER |= GPIO_MODER_MODER15_0; /* GPIOD pin 15 in push-pull mode */
}

static void ToggleGreenLed(void) {
  GPIOD->ODR ^= GPIO_ODR_ODR_12;
}

static void ToggleRedLed(void) {
  GPIOD->ODR ^= GPIO_ODR_ODR_14;
}

static void ToggleBlueLed(void) {
  GPIOD->ODR ^= GPIO_ODR_ODR_15;
}

static void InitRTOSObjects(void) {
  /* Create queues */
  CanRxQueue = xQueueCreate(RECEIVE_QUEUE_SIZE, sizeof(CAN_Rx_t));
  CanTxQueue = xQueueCreate(TRANSMIT_QUEUE_SIZE, sizeof(CAN_Tx_t));

  /* Create tasks */
  xTaskCreate(CANReceiveTask, "", configMINIMAL_STACK_SIZE, NULL, 2u, NULL);
  xTaskCreate(CANTransmitTask, "", configMINIMAL_STACK_SIZE, NULL, 2u, NULL);
  xTaskCreate(MessageSenderTask, "", configMINIMAL_STACK_SIZE, NULL, 1u, NULL);
}
