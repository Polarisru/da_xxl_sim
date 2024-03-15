#ifndef DRV_AC_H
#define DRV_AC_H

#include "defines.h"

#define AC_MAX_CHANNEL    3U

#define AC_MAX_SCALER     64U

void AC_Init(uint8_t channel, uint8_t flen, uint8_t muxpos, uint8_t muxneg, uint8_t scaler, bool enInt);

#endif
