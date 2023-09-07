/*
 * serial.h
 *
 *  Created on: Sep 7, 2023
 *      Author: Isaac
 */

#ifndef INC_SERIAL_MANAGER_H_
#define INC_SERIAL_MANAGER_H_

struct UartMsg {
	char msgID;
	char msgDataLen;
	char msgData[10];
};

#define QUEUE_LEN	5

/* Valid message ID types */
#define MSG_BTN		0x01

/* Other defines for the serial manager functionality */
#define SERIAL_MANAGER_SLEEP_MS 100

/* Function definitions */
int serialManagerInit(void);
void serialManagerTask (void* argument);
void serialSendButtonMessage (char button_index);

#endif /* INC_SERIAL_MANAGER_H_ */
