#ifndef COUNTERS_H
#define COUNTERS_H

#include "defines.h"

void COUNTERS_Reset(uint8_t id);
uint8_t COUNTERS_GetValue(uint8_t id);
void COUNTERS_Process(void);

#endif
