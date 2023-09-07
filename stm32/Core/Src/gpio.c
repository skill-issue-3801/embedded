/*
 * isaac.c
 *
 *  Created on: Sep 2, 2023
 *      Author: Isaac
 */

#include <gpio.h>
#include "main.h"
#include "event_manager.h"

extern UART_HandleTypeDef  huart2;

/* Variables for debouncing  */
uint32_t PB1_LastPress = 0;
uint32_t PB_Green_LastPress = 0;
uint32_t PB_Yellow_LastPress = 0;
uint32_t PB_Orange_LastPress = 0;
uint32_t PB_Purple_LastPress = 0;

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
