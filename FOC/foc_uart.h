#ifndef FOC_UART_H
#define FOC_UART_H

/**
 * @file foc_uart.h
 * @brief 调试串口 TX
 */

#include <stdint.h>

void FOC_UART_Init(void);
void FOC_UART_Write(const uint8_t *data, uint16_t len);

#endif
