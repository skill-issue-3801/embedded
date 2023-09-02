/*
 * isaac.c
 *
 *  Created on: Sep 2, 2023
 *      Author: Isaac
 */

#include "isaac.h"
#include "main.h"

extern UART_HandleTypeDef  huart2;

/* Variables for debouncing  */
uint32_t PB1_LastPress = 0;

/* General serial debugging */
char* led_noise = "Noise?\n\r";
char* led_confirm = "switch!\n\r";

/*
 * This is my temporary playground for any functionality I wish to implement...
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {

	uint32_t tick_now = HAL_GetTick();

	if (GPIO_Pin == B1_Pin) {
		if ((tick_now - PB1_LastPress) > 50) {
			PB1_LastPress = tick_now;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			HAL_UART_Transmit(&huart2, (uint8_t*)led_confirm, 9, HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart2, (uint8_t*)led_noise, 9, HAL_MAX_DELAY);
		}
	}
}
