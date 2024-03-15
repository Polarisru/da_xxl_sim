#ifndef DRV_ADC_H
#define DRV_ADC_H

#include "defines.h"

extern void ADC_Init(uint8_t ref, uint8_t resolution);
extern void ADC_SetChannel(uint8_t channel);
extern bool ADC_IsReady(void);
extern uint16_t ADC_GetResult(void);

#endif
