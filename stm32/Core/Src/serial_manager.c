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

/* Other function dependencies */
#include "stm32l4xx_hal.h"

/* Function definitions */
void serialSend (struct UartMsg *msg);

/* External Variables */
extern UART_HandleTypeDef  huart2;

/* Private Variables */
QueueHandle_t xQueue;

/* ---------- Functions ---------- */
/*
 * @brief	Initialisation function for the serial Manager.
 * @param	None
 * @reval	The success of creation the serial queue, -1 on failure, 0 otherwise.
 */
int serialManagerInit(void) {

	xQueue = xQueueCreate(QUEUE_LEN, sizeof(struct UartMsg));
	if (xQueue == NULL)
		return -1;
	return 0;
}


/*
 * @brief	Task for the serial manager. Reads from the queue and when messages
 * 			are detected they are sent through serial.
 * @param 	argument: Not used.
 * @retval	None.
 */
void serialManagerTask (void* argument) {

	struct UartMsg msg={0};
	BaseType_t ret;

	for (;;) {
		ret = xQueueReceive(xQueue, (void*)&msg, 200);
		if (!ret)
			continue;
		serialSend(&msg);
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
	msg.msgData[1] = 0;
	msg.msgData[2] = 0;

	xQueueSend(xQueue, (void*)&msg, 200);
}


/*
 * @brief	Add a message to the serial queue about a new ADC threshold
 * 			value being reached.
 * @param	adc_index: 		The index of the adc.
 * @param	adc_theshold:	New ADC threshold value.
 * @retval	None.
 */
void serialSendADCMessage (char adc_index, char adc_theshold) {

	struct UartMsg msg;
	msg.msgID = MSG_ADC;
	msg.msgData[0] = adc_index;
	msg.msgData[1] = adc_theshold;
	msg.msgData[2] = 0;

	xQueueSend(xQueue, (void*)&msg, 200);
}

/*
 * @brief	Add a message to the serial queue about a NFC token
 * 			being detected.
 * @param	nfc_chunk0:	0th 8 bit chunk of the NFC ID.
 * @param	nfc_chunk1:	1st 8 bit chunk of the NFC ID.
 * @param	nfc_chunk2: 2nd 8 bit chunk of the NFC ID.
 * @retval	None.
 */
void serialSendNFCMessage (char nfc_chunk0, char nfc_chunk1, char nfc_chunk2) {

	struct UartMsg msg;
	msg.msgID = MSG_NFC;
	msg.msgData[0] = nfc_chunk0;
	msg.msgData[1] = nfc_chunk1;
	msg.msgData[2] = nfc_chunk2;

	xQueueSend(xQueue, (void*)&msg, 200);
}


/*
 * @brief	Convert an incoming message struct and send it over serial.
 * @param	*msg:	Pointer to the UartMsg struct containning the data.
 * @retval	None.
 */
void serialSend (struct UartMsg *msg) {

	char buffer[5] = {0};

	// Looks ugly but works
	buffer[0] = msg->msgID;
	buffer[1] = msg->msgData[0];
	buffer[2] = msg->msgData[1];
	buffer[3] = msg->msgData[2];
	buffer[4] = '\n'; // newline termination for easier python parsing

	// Send that data
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, 5, HAL_MAX_DELAY);

}
