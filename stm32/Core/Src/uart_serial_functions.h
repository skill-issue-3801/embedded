/*
 * uart_serial_functions.h
 *
 *  Created on: Sep 22, 2023
 *      Author: lachjet
 */

#ifndef SRC_UART_SERIAL_FUNCTIONS_H_
#define SRC_UART_SERIAL_FUNCTIONS_H_

void hex_dump(uint8_t *buffer, uint8_t len);
void uart_printf(const char *fmt, ...);

#endif /* SRC_UART_SERIAL_FUNCTIONS_H_ */
