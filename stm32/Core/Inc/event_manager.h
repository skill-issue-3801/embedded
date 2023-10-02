/*
 * event_manager.h
 *
 *  Created on: Sep 7, 2023
 *      Author: Isaac
 */

#ifndef INC_EVENT_MANAGER_H_
#define INC_EVENT_MANAGER_H_

#include <stdint.h>

#define EVENT_MANAGER_SLEEP_MS 100

// Currently 24 bits set, check configUSE_16_BIT_TICKS in FreeRTOSConfig.h -> 0 means 24, 1 means 8
/* Definitions of each of the bits to wait for */
/* Add bits as needed */
#define BITS_WAIT_ALL	0xFFFFF

#define BTN_EVENT_RANGE	0x000FF

#define EVENT_SELECT_USER1	0x01
#define EVENT_SELECT_USER2	0x02
#define EVENT_SELECT_USER3	0x04
#define EVENT_SELECT_USER4	0x08
#define EVENT_VIEW_UP		0x10
#define EVENT_VIEW_DOWN		0x20
#define EVENT_VIEW_LEFT		0x40
#define EVENT_VIEW_RIGHT	0x80

#define VALUE_SELECT_USER1	0x01
#define VALUE_SELECT_USER2 	0x02
#define VALUE_SELECT_USER3 	0x03
#define VALUE_SELECT_USER4	0x04
#define VALUE_VIEW_UP		0x05
#define VALUE_VIEW_DOWN		0x06
#define VALUE_VIEW_LEFT		0x07
#define VALUE_VIEW_RIGHT	0x08

/* Function definitions */
int eventManagerInit (void);
void eventManagerTask (void* argument);
void eventManagerSetBit ();
void eventManagerSetBitISR ();

void trigger_event(uint32_t bits);

#endif /* INC_EVENT_MANAGER_H_ */
