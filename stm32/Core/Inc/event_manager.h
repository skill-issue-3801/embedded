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

#define BTN_EVENT_RANGE	0x000FF

#define PC2_EVENT_BIT	0x01
#define PC3_EVENT_BIT	0x02
#define PC10_EVENT_BIT	0x04
#define PC11_EVENT_BIT	0x08
#define PC12_EVENT_BIT	0x10
#define PC13_EVENT_BIT	0x20
#define PH0_EVENT_BIT	0x40
#define PH1_EVENT_BIT	0x80

#define PC2_VALUE		0x01
#define PC3_VALUE 		0x02
#define PC10_VALUE 		0x03
#define PC11_VALUE		0x04
#define PC12_VALUE		0x05
#define PC13_VALUE		0x06
#define PH0_VALUE		0x07
#define PH1_VALUE		0x08

/* Function definitions */
int eventManagerInit(void);
void eventManagerTask (void* argument);
void eventManagerSetBitISR ();

#endif /* INC_EVENT_MANAGER_H_ */
