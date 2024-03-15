#ifndef WRAPPER_H
#define WRAPPER_H

#include <stdint.h>
#include <stdbool.h>

void SPI_Select(uint8_t dummy);
uint16_t SPI_Receive(uint8_t bits);
bool I2C_ReadReg(uint8_t channel, uint8_t addr, uint8_t reg, uint8_t *rx_data, uint8_t rx_len);
void DRV8320_DoBrake(bool on);
void DRV8320_DoFree(bool on);
void DRV8320_SetDir(bool dir);
bool GPIO_GetPin(uint8_t port, uint8_t pin);
void GPIO_TogglePin(uint8_t port, uint8_t pin);
void HEARTBEAT_Enable(void);
void HEARTBEAT_Disable(void);

#endif
