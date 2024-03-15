#ifndef DRV_I2C_H
#define DRV_I2C_H

#include "defines.h"

#define I2C_REPEATS         3
#define I2C_ACK_TIMEOUT     1000

void I2C_Init(Sercom *channel, uint32_t baudrate);
bool I2C_WriteReg(Sercom *channel, uint8_t addr, uint8_t reg, uint8_t *tx_data, uint8_t tx_len);
bool I2C_ReadReg(Sercom *channel, uint8_t addr, uint8_t reg, uint8_t *rx_data, uint8_t rx_len);

#endif
