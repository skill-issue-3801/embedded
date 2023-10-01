/*
 * event_manager.c
 *
 *  Created on: Sep 7, 2023
 *      Author: Isaac
 */

/* Function Definition Includes */
#include <event_manager.h>

/* FreeRTOS dependencies */
#include "FreeRTOS.h"
#include "event_groups.h"

/* Other function dependencies */
#include "main.h"
#include "stm32l4xx_hal.h"
#include "serial_manager.h"

/* Function definitions */
void handlerButtonBits (EventBits_t bits);
void toggle_user_select (char user_index);

/* Private Variables */
EventGroupHandle_t xEventGroup; // Currently 24 bits set, check configUSE_16_BIT_TICKS in FreeRTOSConfig.h

/* ---------- Functions ---------- */
/*
 * @brief	Initialisation function for the eventManager.
 * @param	None
 * @reval	The success of creation the EventGroup, -1 on failure, 0 otherwise.
 */
int eventManagerInit (void) {

	xEventGroup = xEventGroupCreate();

	if (xEventGroup == NULL)
		return -1;
	 return 0;
}

/*
 * Based off example code provided by FreeRTOS
 * @brief	Task for the event manager. Uses public facing functions to set
 * 			eventGroup bit flags for internal task handling.
 * @param 	argument: Not used.
 * @retval	None.
 */
void eventManagerTask (void* argument) {

	EventBits_t uxBits;
	const TickType_t xTicksToWait = EVENT_MANAGER_SLEEP_MS / portTICK_PERIOD_MS; // Use this if portMAX_DELAY isnt good

	for (;;) {
		uxBits = xEventGroupWaitBits(xEventGroup, BITS_WAIT_ALL, pdTRUE, pdFALSE, xTicksToWait);

		/* Check if the wait bits caught nothing, then resume block */
		if (uxBits == 0x000000) {
			continue;
		}


		/* Add code here for the specific event handling when */
		if (uxBits & BTN_EVENT_RANGE) {
			// Call button handling code here
			handlerButtonBits(uxBits);
		}
	}
}

/*
 *@brief	Outward facing bit setting function for other files to interact
 * 			with the event manager via the bit flags. Uses inbuilt eventGroup
 * 			safeties to ensure atomic actions and no race conditions.
 * 			Variant for non-ISR specific operations.
 * @param	bits:	The bit flag for the event (refer to event_manager.h
 * 					for bits flag definitions).
 * @retval	None.
 */
void eventManagerSetBit (EventBits_t bits) {

	xEventGroupSetBits(xEventGroup, bits);
}

/*
 * @brief	Outward facing bit setting function for other files to interact
 * 			with the event manager via the bit flags. Uses inbuilt eventGroup
 * 			safeties to ensure atomic actions and no race conditions.
 * 			Variant for ISR specific operations.
 * @param	bits:	The bit flag for the event (refer to event_manager.h
 * 					for bits flag definitions).
 * @retval	None.
 */
void eventManagerSetBitISR (EventBits_t bits) {

	BaseType_t xHigherPriorityTaskWoken, xResult;
	xHigherPriorityTaskWoken = pdFALSE;
	xResult = xEventGroupSetBitsFromISR(xEventGroup, bits, &xHigherPriorityTaskWoken);

	if (xResult != pdFAIL)
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * @brief	Function for handling the button bits and passing information
 * 			to the serial output.
 * @param 	bits: The bits to be checked.
 * @retval	None.
 */
void handlerButtonBits (uint32_t bits) {

	bits = bits & BTN_EVENT_RANGE;

	// Check our user selection first, set any LEDs as appropriate
	if (bits & EVENT_SELECT_USER1) {
		serialSendButtonMessage(VALUE_SELECT_USER1);
		toggle_user_select(1);
	}
	if (bits & EVENT_SELECT_USER2) {
		serialSendButtonMessage(VALUE_SELECT_USER2);
		toggle_user_select(2);
	}
	if (bits & EVENT_SELECT_USER3) {
		serialSendButtonMessage(VALUE_SELECT_USER3);
		toggle_user_select(3);
	}
	if (bits & EVENT_SELECT_USER4) {
		serialSendButtonMessage(VALUE_SELECT_USER4);
		toggle_user_select(4);
	}

	// Check our view control buttons next
	if (bits & EVENT_VIEW_UP)
		serialSendButtonMessage(VALUE_VIEW_UP);
	if (bits & EVENT_VIEW_DOWN)
		serialSendButtonMessage(VALUE_VIEW_DOWN);
	if (bits & EVENT_VIEW_LEFT)
		serialSendButtonMessage(VALUE_VIEW_LEFT);
	if (bits & EVENT_VIEW_RIGHT)
		serialSendButtonMessage(VALUE_VIEW_RIGHT);
}

/**
 * @brief	GPIO control function for toggling user LED operation. Shows
 * 			which user has been "selected"
 * @param	user_index:	The index of the user to be set. Positions are indexed
 * 						1 to 4
 * @retval	None.
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
