/*
 * event_manager.h
 *
 *  Created on: Sep 7, 2023
 *      Author: Isaac
 */

#ifndef INC_EVENT_MANAGER_H_
#define INC_EVENT_MANAGER_H_

#define EVENT_MANAGER_SLEEP_MS 100

// Currently 24 bits set, check configUSE_16_BIT_TICKS in FreeRTOSConfig.h -> 0 means 24, 1 means 8
/* Definitions of each of the bits to wait for */
/* Add bits as needed */
#define BITS_WAIT_ALL	0xFFFFF

#define BTN_EVENT_RANGE	0x0001F

#define BTN1_EVENT_BIT	0x01
#define BTN2_EVENT_BIT 	0x02
#define BTN3_EVENT_BIT 	0x04
#define BTN4_EVENT_BIT 	0x08
#define BTN5_EVENT_BIT	0x10

#define BTN1_VALUE	0x01
#define BTN2_VALUE 	0x02
#define BTN3_VALUE 	0x03
#define BTN4_VALUE	0x04
#define BTN5_VALUE	0x05

/* Function definitions */
int eventManagerInit(void);
void eventManagerTask (void* argument);
void eventManagerSetBitISR ();

#endif /* INC_EVENT_MANAGER_H_ */
