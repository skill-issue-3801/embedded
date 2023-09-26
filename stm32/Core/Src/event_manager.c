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
#include "stm32l4xx_hal.h"
#include "serial_manager.h"

/* Function definitions */
void handlerButtonBits (EventBits_t bits);

/* Private Variables */
EventGroupHandle_t xEventGroup; // Currently 24 bits set, check configUSE_16_BIT_TICKS in FreeRTOSConfig.h

/* ---------- Functions ---------- */
/*
 * @brief	Initialisation function for the eventManager.
 * @param	None
 * @reval	The success of creation the EventGroup, -1 on failure, 0 otherwise.
 */
int eventManagerInit(void) {

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
 * @brief	Outward facing bit setting function for other files to interact
 * 			with the event manager via the bit flags. Uses inbuilt eventGroup
 * 			safeties to ensure atomic actions and no race conditions.
 * 			Variant for ISR specific sets.
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

	if (bits & PC2_EVENT_BIT)
		serialSendButtonMessage(PC2_VALUE);
	if (bits & PC3_EVENT_BIT)
		serialSendButtonMessage(PC3_VALUE);
	if (bits & PC10_EVENT_BIT)
		serialSendButtonMessage(PC10_VALUE);
	if (bits & PC11_EVENT_BIT)
		serialSendButtonMessage(PC11_VALUE);
	if (bits & PC12_EVENT_BIT)
		serialSendButtonMessage(PC12_VALUE);
	if (bits & PC13_EVENT_BIT)
		serialSendButtonMessage(PC13_VALUE);
	if (bits & PH0_EVENT_BIT)
		serialSendButtonMessage(PH0_VALUE);
	if (bits & PH1_EVENT_BIT)
		serialSendButtonMessage(PH1_VALUE);
}
