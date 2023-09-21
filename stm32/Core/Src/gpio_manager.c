/*
 * isaac.c
 *
 *  Created on: Sep 2, 2023
 *      Author: Isaac
 */

/* Function Definition Includes */
#include <gpio_manager.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "main.h"
#include "event_manager.h"

/* External Variables */
extern UART_HandleTypeDef  	huart2; // Only included for debug, remove in the future please
extern ADC_HandleTypeDef 	hadc1;

/* Function definitions */
float gpio_adc_calcvalue(uint32_t raw_adc, float vdd);
void fading_average(float* storage, float new_value, float alpha);

/* Private Variables */
uint32_t PB1_LastPress = 0;
uint32_t PB_Green_LastPress = 0;
uint32_t PB_Yellow_LastPress = 0;
uint32_t PB_Orange_LastPress = 0;
uint32_t PB_Purple_LastPress = 0;
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
 * @brief	Task for the gpio manager. Reads the ADC values, if the value
 * 			has changed
 */
void gpioManagerTask (void* argument) {

	const TickType_t msDelay = 100 / portTICK_PERIOD_MS;
	const float fade_alpha = 0.9;
	char output[40] = {0}; // Remove this, temp debug printing

	// Fading average variables
	float adc_pc0_avg = 0;
	float adc_pc1_avg = 0;
	uint32_t sample;

	for (;;) {

		sample = gpio_adc_calcvalue(adc_values[0], ADC_PC0_VDD);
		fading_average(&adc_pc0_avg, sample, fade_alpha);

		sprintf(output, "ADC on PC0=%.1f\n\r", adc_pc0_avg);
//		val2 = adc_values[1];
		HAL_UART_Transmit(&huart2, (uint8_t*)output, 16, HAL_MAX_DELAY);


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

	if (GPIO_Pin == B1_Pin) {
		if ((tick_now - PB1_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(BIT_BTN1);
			PB1_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_GREEN_Pin) {
		if ((tick_now - PB_Green_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(BIT_BTN2);
			PB_Green_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_YELLOW_Pin) {
		if ((tick_now - PB_Yellow_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(BIT_BTN3);
			PB_Yellow_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_ORANGE_Pin) {
		if ((tick_now - PB_Orange_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(BIT_BTN4);
			PB_Orange_LastPress = tick_now;
		}
	} else if (GPIO_Pin == PB_PURPLE_Pin) { // Could swap to pure else, leaving as if just in case other PBs are used.
		if ((tick_now - PB_Purple_LastPress) > PB_DEBOUNCE_THRESH_MS) {
			eventManagerSetBitISR(BIT_BTN5);
			PB_Purple_LastPress = tick_now;
		}
	}
}
