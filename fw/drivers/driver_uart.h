#ifndef DRV_UART_H
#define DRV_UART_H

#include "defines.h"

/**
 * \brief Parity settings
 */
enum uart_parity_mode { UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD };

/**
 * \brief Stop bit settings
 */
enum uart_stop_mode { UART_STOP_1BIT, UART_STOP_2BITS };

void UART_Init(Sercom *channel, uint8_t pinRXPO, uint8_t pinTXPO, uint32_t baud,
               enum uart_stop_mode stop, enum uart_parity_mode parity);
void UART_DeInit(Sercom *channel);
void UART_SetBaudrate(Sercom *channel, uint32_t baud);
void UART_SendByte(Sercom *channel, uint8_t data);
bool UART_HaveData(Sercom *channel);
uint8_t UART_GetByte(Sercom *channel);

#endif
