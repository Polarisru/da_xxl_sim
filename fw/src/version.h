#ifndef VERSION_H
#define VERSION_H

#include "defines.h"

uint16_t* VERSION_GetHW(void);
uint16_t* VERSION_GetFW(void);
char* VERSION_GetStringFW(void);
char VERSION_GetRevisionStr(uint8_t pos);

#endif
