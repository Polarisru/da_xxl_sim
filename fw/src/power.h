#ifndef POWER_H
#define POWER_H

#include "defines.h"

void AC_Handler(void);
void POWER_RestartApp(void);
bool POWER_CheckSupply(void);
void POWER_Configuration(void);

#endif
