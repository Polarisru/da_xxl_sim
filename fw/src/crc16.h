#ifndef CRC16_H
#define CRC16_H

#include "defines.h"

#define CRC16_INIT_VAL      0xFFFFU

uint16_t CRC16_Calc(uint16_t init, uint8_t *data, uint16_t len);
uint16_t CRC16_CalcCCITT(uint8_t *data, uint16_t len);

#endif
