/*
 * isaac.h
 *
 *  Created on: Sep 2, 2023
 *      Author: Isaac
 */

#ifndef INC_GPIO_MANAGER_H_
#define INC_GPIO_MANAGER_H_

#define PB_DEBOUNCE_THRESH_MS	200
#define ADC_COUNT				2

int gpioManagerInit(void);
void gpioManagerTask (void* argument);

#endif /* INC_GPIO_MANAGER_H_ */
