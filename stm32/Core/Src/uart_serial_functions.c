/*
 * uart_serial_functions.c
 *
 *  Created on: Sep 22, 2023
 *      Author: lachjet
 */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include "main.h"


void hex_dump(uint8_t* buffer, uint8_t len) {
	char buf[10];
	memset(buf, 0, 5);
	for (int i = 0; i < len; i++) {
		if (i % 10 == 0) {
			uart_printf("\r\n");
		}
		uart_printf("0x%0X ", buffer[i]);
	}
	uart_printf("\r\n");
}

void uart_printf(const char *fmt, ...) {
	char buf[100];
	memset(buf, 0, sizeof(buf));

	va_list args;

	va_start(args, fmt);

	vsprintf(buf, fmt, args);

	va_end(args);

	int i = 0;
	while (i < 100) {
		if (buf[i] == '\0' || buf[i] == '\n') {
			i++;
			break;
		}
		i++;
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)buf, (sizeof(char) * i), HAL_MAX_DELAY);
}
