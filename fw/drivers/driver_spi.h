#ifndef DRV_SPI_H
#define DRV_SPI_H

#include "defines.h"

#define SPI_BAUDRATE    500000UL

enum
{
  SPI_SELECT_NONE,
  SPI_SELECT_MAGNET,
  SPI_SELECT_DRIVER
};

void SPI_Init(void);
void SPI_Select(uint8_t device);
void SPI_Transmit(uint16_t data, uint8_t bits);
uint16_t SPI_Receive(uint8_t bits);

#endif
