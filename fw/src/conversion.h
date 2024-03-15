#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include "defines.h"

uint8_t  CONVERSION_CalcTemperature(uint16_t value);
uint8_t  CONVERSION_CalcVoltage(uint16_t value);
uint8_t  CONVERSION_CalcCurrent(uint16_t value);
uint16_t CONVERSION_CalcDegreesFromPos(int16_t value);
int16_t  CONVERSION_CalcPosFromDegrees(uint16_t position);
int16_t  CONVERSION_CalcPosFromSetPos170(uint8_t byte1, uint8_t byte2);
int16_t  CONVERSION_CalcPosFromSetPos100(uint8_t byte1, uint8_t byte2);
uint16_t CONVERSION_CalcResponseForSetPos100(int16_t position);
int16_t  CONVERSION_CalcPosFromSetPosExt(uint8_t byte1, uint8_t byte2);
uint16_t CONVERSION_CalcResponseForSetPosExt(int16_t position);
uint8_t  CONVERSION_BuildOldStatus(uint16_t status);
uint8_t  CONVERSION_BuildVolzStatus(uint16_t status);

#endif
