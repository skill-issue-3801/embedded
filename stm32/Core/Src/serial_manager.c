/*
 * serial.c
 *
 *  Created on: Sep 7, 2023
 *      Author: Isaac
 */

/* Function Definition Includes */
#include <serial_manager.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "cmsis_os2.h"
/* Other function dependencies */
#include "stm32l4xx_hal.h"

/* Function definitions */
void serialParseButton (struct UartMsg *msg);

/* External Variables */
extern UART_HandleTypeDef  huart2;

/* Private Variables */
QueueHandle_t xQueue;
osMessageQueueId_t newQueue;

/* ---------- Functions ---------- */
/*
 * @brief	Initialisation function for the serial Manager.
 * @param	None
 * @reval	The success of creation the serial queue, -1 on failure, 0 otherwise.
 */
int serialManagerInit(void) {

	newQueue = osMessageQueueNew (QUEUE_LEN, sizeof(struct UartMsg), NULL);
	if (newQueue == NULL)
		return -1;
	return 0;
}


/*
 * @brief	Task for the event manager. Uses public facing functions to set
 * 			eventGroup bit flags for internal task handling.
 * @param 	argument: Not used.
 * @retval	None.
 */
void serialManagerTask (void* argument) {

	struct UartMsg msg={0};
	osStatus_t ret;

	for (;;) {
		ret = osMessageQueueGet(newQueue, &msg, NULL, 200);
		if (ret == osErrorTimeout)
			continue;
		if (msg.msgID == MSG_BTN)
			serialParseButton(&msg);
	}
}


/*
 * @brief	Add a message to the serial queue about a button being pressed.
 * @param	button_index: The index of the button pressed.
 * @retval	None.
 */
void serialSendButtonMessage (char button_index) {

	struct UartMsg msg;
	msg.msgID = MSG_BTN;
	msg.msgData[0] = button_index;
	msg.msgDataLen = 1;

	osMessageQueuePut(newQueue, &msg, 0, 200);
}


/*
 * @brief	Function that parses button information and sends
 * 			the appropriate information over serial.
 * @param	msg: Pointer to a UartMsg structure.
 * @retval	None.
 */
void serialParseButton (struct UartMsg *msg) {

	// This wouldnt be here, it would just send the uart construct
	// this is just for debugging purposes.
	if (msg->msgData[0] == 0x00001) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"main\n\r", 6, HAL_MAX_DELAY);
	} else if (msg->msgData[0] == 0x00002) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"green\n\r", 7, HAL_MAX_DELAY);
	} else if (msg->msgData[0] == 0x00004) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"yellow\n\r", 8, HAL_MAX_DELAY);
	} else if (msg->msgData[0] == 0x00008) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"orange\n\r", 8, HAL_MAX_DELAY);
	} else if (msg->msgData[0] == 0x00010) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"purple\n\r", 8, HAL_MAX_DELAY);
	}
}
