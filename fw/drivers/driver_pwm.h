#ifndef DRV_PWM_H
#define DRV_PWM_H

#include "defines.h"

void PWM_Init(void);
void PWM_Enable(uint8_t channel);
void PWM_Disable(uint8_t channel);
void PWM_Set(uint8_t value);

#endif
