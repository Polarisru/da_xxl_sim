#ifndef UTILS_H
#define UTILS_H

#include "defines.h"

uint8_t UTILS_GetLength(char *str);
int8_t UTILS_Sign(int32_t x);
int16_t UTILS_LimitValue(int32_t in, uint16_t limit);
char *UTILS_IntToStr(uint32_t value, uint8_t min_len);
char *UTILS_FloatToStr(float value, uint8_t pos);

#endif
