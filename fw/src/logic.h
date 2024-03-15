#ifndef LOGIC_H
#define LOGIC_H

#include "defines.h"

void LOGIC_RestartApp(void);
void LOGIC_ResetStatusBits(void);
void LOGIC_SetDefaultValues(uint8_t motor_mode, uint8_t reset_mode);
void LOGIC_ResetTimeout(uint8_t type);
//void LOGIC_ProcessLoadCounters(void);
void LOGIC_ResetPWM(void);
void LOGIC_Task(void *pParameters);

#endif
