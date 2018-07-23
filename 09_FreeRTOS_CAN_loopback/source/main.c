/* STD lib */
#include <string.h>

/* CMSIS */
#include "stm32f4xx.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* User */
#include "RCC.h"
#include "CAN.h"


/******************************************************************************/
/* Definitions ****************************************************************/
/******************************************************************************/
#define CAN_RECEIVE_QUEUE_SIZE      10u
#define CAN_TRANSMIT_QUEUE_SIZE     10u
#define CAN_TRANSMISSION_DELAY      1000u
#define CAN_RETRANSMISSION_DELAY    1u
#define CAN_MESSAGE_DATA_LENGTH     8u


/******************************************************************************/
/* Private variables **********************************************************/
/******************************************************************************/
static volatile xQueueHandle CANReceiveQueue;
static volatile xQueueHandle CANTransmitQueue;


/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
static void InitAll(void);
static void InitHardware(void);
static void InitLEDPins(void);
static void ToggleGreenED(void);
static void ToggleRedLED(void);
static void ToggleBlueLED(void);
static void InitFreeRTOSObjects(void);


/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
/**
 * CAN receive IRQ handler.
 * Receive messages from CAN and send them through queue to CAN receive task.
 */
void CAN1_RX0_IRQHandler(void) {
    portBASE_TYPE higher_priority_task_woken;
    CAN_RX_message_t CAN_RX_message;    /* Received CAN message */

    /* Receive CAN message from all CAN RX mailboxes */
    while (CAN_Receive(&CAN_RX_message)) {
        /* Send message to CAN receive task */
        higher_priority_task_woken = pdFALSE;
        xQueueSendFromISR(CANReceiveQueue, &CAN_RX_message,
                          &higher_priority_task_woken);
        if (higher_priority_task_woken == pdTRUE) {
            taskYIELD();
        }
    }
}


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
    CAN_RX_message_t CAN_RX_message;    /* Received CAN message */

    while (1u) {
        /* Receive CAN message from CAN IRQ */
        xQueueReceive(CANReceiveQueue, &CAN_RX_message, portMAX_DELAY);
        /* Indicate message receiving from CAN */
        ToggleGreenED();

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
    CAN_TX_message_t CAN_TX_message;    /* CAN message to send */

    while (1u) {
        /* Receive CAN message from queue */
        xQueueReceive(CANTransmitQueue, &CAN_TX_message, portMAX_DELAY);
        /* Indicate message receiving from queue */
        ToggleBlueLED();

        /* Try to put CAN message in mailbox */
        while (!CAN_Transmit(&CAN_TX_message)) {
            /* CAN retransmission delay if mailboxes are busy */
            vTaskDelay(CAN_RETRANSMISSION_DELAY);
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
    CAN_TX_message_t CAN_TX_message;    /* CAN message to send */
    uint8_t data = 0u;                  /* Value to send */

    while (1u) {
        memset(&CAN_TX_message, 0u, sizeof(CAN_TX_message_t));
        /* Fill CAN message data length */
        CAN_TX_message.TDTR |= CAN_MESSAGE_DATA_LENGTH;
        /* Fill CAN low data */
        CAN_TX_message.TDLR |= data;
        CAN_TX_message.TDLR |= (data + 1u) << 8u;
        CAN_TX_message.TDLR |= (data + 2u) << 16u;
        CAN_TX_message.TDLR |= (data + 3u) << 24u;
        /* Fill CAN high data */
        CAN_TX_message.TDHR |= (data + 4u);
        CAN_TX_message.TDHR |= (data + 5u) << 8u;
        CAN_TX_message.TDHR |= (data + 6u) << 16u;
        CAN_TX_message.TDHR |= (data + 7u) << 24u;
        /* Change value to send */
        ++data;

        /* Send CAN message through queue to CAN transmit task */
        xQueueSend(CANTransmitQueue, &CAN_TX_message, 0u);
        /* Indicate message sending */
        ToggleRedLED();

        /* Delay to next message */
        vTaskDelay(CAN_TRANSMISSION_DELAY);
    }
}




/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
    InitAll();
    vTaskStartScheduler();

    while (1u) {

    }
}




/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
static void InitAll(void) {
    InitHardware();
    InitFreeRTOSObjects();
}


static void InitHardware(void) {
    SystemClock_Init();
    InitLEDPins();
    CAN_Init();
}


static void InitLEDPins(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;    /* Enable GPIOD clock */
    GPIOD->MODER |= GPIO_MODER_MODER12_0;   /* GPIOD pin 12 in push-pull mode */
    GPIOD->MODER |= GPIO_MODER_MODER14_0;   /* GPIOD pin 14 in push-pull mode */
    GPIOD->MODER |= GPIO_MODER_MODER15_0;   /* GPIOD pin 15 in push-pull mode */
}


static void ToggleGreenED(void) {
    GPIOD->ODR ^= GPIO_ODR_ODR_12;  /* Toggle GPIOD pin 12 */
}


static void ToggleRedLED(void) {
    GPIOD->ODR ^= GPIO_ODR_ODR_14;  /* Toggle GPIOD pin 14 */
}


static void ToggleBlueLED(void) {
    GPIOD->ODR ^= GPIO_ODR_ODR_15;  /* Toggle GPIOD pin 15 */
}


static void InitFreeRTOSObjects(void) {
    /* Create queues */
    CANReceiveQueue = xQueueCreate(CAN_RECEIVE_QUEUE_SIZE,
                                   sizeof(CAN_RX_message_t));
    CANTransmitQueue = xQueueCreate(CAN_TRANSMIT_QUEUE_SIZE,
                                    sizeof(CAN_TX_message_t));

    /* Create RTOS tasks */
    xTaskCreate(CANReceiveTask, (char const*)"", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 2u, NULL);
    xTaskCreate(CANTransmitTask, (char const*)"", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 2u, NULL);
    xTaskCreate(MessageSenderTask, (char const*)"", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1u, NULL);
}
