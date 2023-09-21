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

#define GPIO_MANAGER_SLEEP_MS 	100

// ADC relevant defines
#define ADC_12BIT_MAX_VAL		4095
#define ADC_VDD					5.0
#define ADC_THRESHOLDS			5

#define ADC_PC0_INDX			0
#define ADC_PC1_INDX			1

int gpioManagerInit(void);
void gpioManagerTask (void* argument);


#endif /* INC_GPIO_MANAGER_H_ */
