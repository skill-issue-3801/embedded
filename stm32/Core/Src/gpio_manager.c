/*
 * isaac.c
 *
 *  Created on: Sep 2, 2023
 *      Author: Isaac
 */

/* Function Definition Includes */
#include <gpio_manager.h>
#include "serial_manager.h"
#include "event_manager.h"

/* Other dependencies */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"


/* External Variables */
extern ADC_HandleTypeDef 	hadc1;

/* Function definitions */
float gpio_adc_calcvalue(uint32_t raw_adc, float vdd);
void fading_average(float* storage, float new_value, float alpha);
void adc_threshold (float value, float *prev_thresh, char index);

/* Private Variables */
uint32_t PC2_LastPress 	= 0;
uint32_t PC3_LastPress 	= 0;
uint32_t PC10_LastPress = 0;
uint32_t PC11_LastPress = 0;
uint32_t PC12_LastPress = 0;
uint32_t PC13_LastPress = 0;
uint32_t PH0_LastPress 	= 0;
uint32_t PH1_LastPress 	= 0;
uint32_t adc_values[ADC_COUNT];

/* ADC enumerator */
enum Threshold {
	THRESH0,
	THRESH1,
	THRESH2,
	THRESH3,
	THRESH4,
	THRESH5
};

/*
 * @brief	Initialise the ADC for future GPIO interactions.
 * @param	None.
 * @return	0 on success, 1 otherwise
 */
int gpioManagerInit(void) {

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED))
		return 1;
	if (HAL_ADC_Start_DMA(&hadc1, adc_values, ADC_COUNT))
		return 1;
	return 0;
}

/*
 * @brief	Task for the gpio manager. Reads the ADC values, and performs
 * 			a fading average on the value. Determines which threshold that
 * 			value sits in, and if the threshold has changed, alerts the
 * 			serial manager.
 * @param	argument	Unused.
 * @return	None.
 */
void gpioManagerTask (void* argument) {

	// Const defines for operation
	const TickType_t msDelay = GPIO_MANAGER_SLEEP_MS / portTICK_PERIOD_MS;
	const float fade_alpha = 0.8;

	// Fading average variables
	float pc0_adc_avg = 0;
	float pc1_adc_avg = 0;
	float pc0_thresh = THRESH5;
	float pc1_thresh = THRESH5;

	// Variables for performing operations, made for eye considerations
	uint32_t sample;

	for (;;) {
		// Calculate ADC values
		sample = gpio_adc_calcvalue(adc_values[0], ADC_VDD);
		fading_average(&pc0_adc_avg, sample, fade_alpha);

		sample = gpio_adc_calcvalue(adc_values[1], ADC_VDD);
		fading_average(&pc1_adc_avg, sample, fade_alpha);

		// Find if an ADC value has entered a new threshold
		adc_threshold(pc0_adc_avg, &pc0_thresh, ADC_PC0_INDX);
		adc_threshold(pc1_adc_avg, &pc1_thresh, ADC_PC1_INDX);

		// Get new ADC values and sleep this task.
		HAL_ADC_Start_DMA(&hadc1, adc_values, ADC_COUNT);
		vTaskDelay(msDelay);
	}
}


/*
 * @brief	Find the voltage value of the ADC.
 * @param	raw_adc	The raw ADC reading, we are expecting readings
 * 					from 12 bit sampels
 * @param	vdd		Supply voltage the ADC is using
 * @return	The voltage of the adc sample.
 */
float gpio_adc_calcvalue (uint32_t raw_adc, float vdd) {

	return (raw_adc / 4095.0 * vdd);
}

/*
 * @brief	Take a given ADC voltage sample and perform a fading
 * 			average on it.
 * @param	*storage	Pointer to the variable that stores the fading average.
 * @param	new_value	Latest value to be integrated into the average.
 * @param	alpha		The fading factor to utilise.
 * @return	None.
 */
void fading_average (float* storage, float new_value, float alpha) {

	*storage = new_value*alpha + (1.0-alpha)*storage[0];
}

/*
 * @brief	Find the threshold the ADC value sits in. If the new threshold is
 * 			different to the previous threshold, send new threshold via
 * 			serial.
 * @param	value: The current float value of the ADC value.
 * @param	prev_thresh: Pointer to the previous threshold the ADC value occupied.
 * @param	index: Index of the ADC being checked.
 * @return	None.
 */
void adc_threshold (float value, float *prev_thresh, char index) {

	float thresh_now = 0;

	// Find the threshold the ADC value occupies.
	if (THRESH_4_UPPER < value && value <= THRESH_5_UPPER)
		thresh_now = THRESH5;
	else if (THRESH_3_UPPER < value && value <= THRESH_4_UPPER)
		thresh_now = THRESH4;
	else if (THRESH_2_UPPER < value && value <= THRESH_3_UPPER)
		thresh_now = THRESH3;
	else if (THRESH_1_UPPER < value && value <= THRESH_2_UPPER)
		thresh_now = THRESH2;
	else if (THRESH_0_UPPER < value && value <= THRESH_1_UPPER)
		thresh_now = THRESH1;
	else
		thresh_now = THRESH0;

	// Check if different
	if (thresh_now != *prev_thresh) {
		// Different thresh found, set the tracker variable and send via
		*prev_thresh = thresh_now;
		serialSendADCMessage(index, thresh_now);
	}
}


/*
 * @brief	Interrupt callback for associated GPIO pins. Used to handle
 * 			set eventGroup flags appropriate to the button. Software
 * 			debouncing included.
 * @param	GPIO_Pin: Index of the GPIO_Pin.
 * @return	None.
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {

	uint32_t tick_now = HAL_GetTick();

	if (GPIO_Pin == PB_PC2_Pin) {
		if ((tick_now - PC2_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(EVENT_VIEW_UP);
			PC2_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC3_Pin) {
		if ((tick_now - PC3_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(EVENT_VIEW_DOWN);
			PC3_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC10_Pin) {
		if ((tick_now - PC10_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(EVENT_SELECT_USER1);
			PC10_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC11_Pin) {
		if ((tick_now - PC11_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(EVENT_SELECT_USER2);
			PC11_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC12_Pin) {
		if ((tick_now - PC12_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(EVENT_SELECT_USER3);
			PC12_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC13_Pin) {
		if ((tick_now - PC13_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(EVENT_SELECT_USER4);
			PC13_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PH0_Pin) {
		if ((tick_now - PH0_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(EVENT_VIEW_LEFT);
			PH0_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PH1_Pin) {
		if ((tick_now - PH1_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(EVENT_VIEW_RIGHT);
			PH1_LastPress = tick_now;
		}
	}
}
