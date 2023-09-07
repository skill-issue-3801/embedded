/*
 * event_manager.h
 *
 *  Created on: Sep 7, 2023
 *      Author: Isaac
 */

#ifndef INC_EVENT_MANAGER_H_
#define INC_EVENT_MANAGER_H_

// Currently 24 bits set, check configUSE_16_BIT_TICKS in FreeRTOSConfig.h -> 0 means 24, 1 means 8
/* Definitions of each of the bits to wait for */
/* Add bits as needed */
#define BITS_WAIT_ALL	0xFFFFF

#define BIT_RANGE_BTN	0x0001F

#define BIT_BTN1		0x00001
#define BIT_BTN2 		0x00002
#define BIT_BTN3 		0x00004
#define BIT_BTN4 		0x00008
#define BIT_BTN5		0x00010



/* Function definitions */
int eventManagerInit(void);
void eventManagerTask (void* argument);
void eventManagerSetBitISR ();

#endif /* INC_EVENT_MANAGER_H_ */
