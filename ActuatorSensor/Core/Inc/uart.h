/*
 * uart.h
 *
 *  Created on: Mar 23, 2023
 *      Author: PC
 */

#ifndef LIB_PERIFERIJE_UART_UART_H_
#define LIB_PERIFERIJE_UART_UART_H_

#include <stdint.h>
#include <stdbool.h>

void uart_init();

void uart_send_byte1 (uint8_t data);
void uart_send_byte2(uint8_t data);
void uart_send_byte6 (uint8_t data);
void uart_send_str (char * str);

void USART6_IRQHandler (void);
void USART1_IRQHandler (void);
void USART2_IRQHandler (void);

void move_ax (uint8_t id, uint16_t angle, uint16_t brzina);
void read_ax (uint8_t id);

void write_to_buffer1(uint8_t data);
void write_to_buffer2(uint8_t data);
void write_to_buffer6(uint8_t data);

uint16_t get_angle();
uint16_t get_x_uart();
uint16_t get_y_uart();

#endif /* LIB_PERIFERIJE_UART_UART_H_ */
