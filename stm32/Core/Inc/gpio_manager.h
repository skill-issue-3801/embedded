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

// ADC relevant defines
#define ADC_12BIT_MAX_VAL		4096
#define ADC_PC0_VDD				5.0
#define ADC_PC1_VDD				3.3
#define ADC1_THRESHOLDS			5
#define ADC2_THESHOLDS			3

int gpioManagerInit(void);
void gpioManagerTask (void* argument);


#endif /* INC_GPIO_MANAGER_H_ */
