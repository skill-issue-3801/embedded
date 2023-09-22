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
	char msgData[3];
};

#define QUEUE_LEN	5

/* Valid message ID types */
#define MSG_BTN		0x01
#define MSG_ADC		0x02
#define MSG_NFC		0x03

/* Other defines for the serial manager functionality */
#define SERIAL_MANAGER_SLEEP_MS 100

/* Function definitions */
int serialManagerInit(void);
void serialManagerTask (void* argument);
void serialSendButtonMessage (char button_index);
void serialSendADCMessage (char adc_index, char adc_theshold);
void serialSendNFCMessage (char nfc_chunk0, char nfc_chunk1, char nfc_chunk2);

#endif /* INC_SERIAL_MANAGER_H_ */
