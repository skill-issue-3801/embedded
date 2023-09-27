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
	const float thresh_width = ADC_VDD / ADC_THRESHOLDS;

	// Fading average variables
	float pc0_adc_avg = 0;
	float pc1_adc_avg = 0;
	float pc0_thresh = -1;
	float pc1_thresh = -1;

	// Variables for performing operations, made for eye considerations
	uint32_t sample;
	float sample_thresh;

	for (;;) {
		// Calculate ADC values
		sample = gpio_adc_calcvalue(adc_values[0], ADC_VDD);
		fading_average(&pc0_adc_avg, sample, fade_alpha);

		sample = gpio_adc_calcvalue(adc_values[1], ADC_VDD);
		fading_average(&pc1_adc_avg, sample, fade_alpha);

		// Find if an ADC value has entered a new threshold
		sample_thresh = floorf(pc0_adc_avg / thresh_width);
		if (sample_thresh != pc0_thresh) {
			pc0_thresh = sample_thresh;
			serialSendADCMessage(ADC_PC0_INDX, pc0_thresh);
		}

		sample_thresh = floorf(pc1_adc_avg / thresh_width);
		if (sample_thresh != pc1_thresh) {
			pc1_thresh = sample_thresh;
			serialSendADCMessage(ADC_PC1_INDX, pc1_thresh);
		}

		HAL_ADC_Start_DMA(&hadc1, adc_values, ADC_COUNT);
		vTaskDelay(msDelay);
	}
}

/**
 * @brief
 */
void toggle_user_select (char user_index) {

	switch (user_index) {
	case 1 :
		HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
		break;
	case 2 :
		HAL_GPIO_TogglePin(USER_LED_2_GPIO_Port, USER_LED_2_Pin);
		break;
	case 3 :
		HAL_GPIO_TogglePin(USER_LED_3_GPIO_Port, USER_LED_3_Pin);
		break;
	case 4 :
		HAL_GPIO_TogglePin(USER_LED_4_GPIO_Port, USER_LED_4_Pin);
		break;
	default :
		break;
	}
}

/*
 * @brief	Find the voltage value of the ADC.
 * @param	raw_adc	The raw ADC reading, we are expecting readings
 * 					from 12 bit sampels
 * @param	vdd		Supply voltage the ADC is using
 * @return	The voltage of the adc sample.
 */
float gpio_adc_calcvalue(uint32_t raw_adc, float vdd) {

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
void fading_average(float* storage, float new_value, float alpha) {

	*storage = new_value*alpha + (1.0-alpha)*storage[0];
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
			eventManagerSetBitISR(PC2_EVENT_BIT);
			PC2_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC3_Pin) {
		if ((tick_now - PC3_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(PC3_EVENT_BIT);
			PC3_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC10_Pin) {
		if ((tick_now - PC10_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(PC10_EVENT_BIT);
			PC10_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC11_Pin) {
		if ((tick_now - PC11_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(PC11_EVENT_BIT);
			PC11_LastPress = tick_now;
			toggle_user_select(1);
		}
	} else if (GPIO_Pin == PB_PC12_Pin) {
		if ((tick_now - PC12_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(PC12_EVENT_BIT);
			PC12_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PC13_Pin) {
		if ((tick_now - PC13_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(PC13_EVENT_BIT);
			PC13_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PH0_Pin) {
		if ((tick_now - PH0_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(PH0_EVENT_BIT);
			PH0_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PH1_Pin) {
		if ((tick_now - PH1_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(PH1_EVENT_BIT);
			PH1_LastPress = tick_now;
		}
	}
}
