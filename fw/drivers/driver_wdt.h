#ifndef DRV_WDT_H
#define DRV_WDT_H

#include "defines.h"

enum {
  WDT_PER_8CYCLES,
  WDT_PER_16CYCLES,
  WDT_PER_32CYCLES,
  WDT_PER_64CYCLES,
  WDT_PER_128CYCLES,
  WDT_PER_256CYCLES,
  WDT_PER_512CYCLES,
  WDT_PER_1024CYCLES,
  WDT_PER_2048CYCLES,
  WDT_PER_4096CYCLES,
  WDT_PER_8192CYCLES,
  WDT_PER_16384CYCLES,
  WDT_PER_LAST
};

extern void WDT_Init(uint8_t period);
extern void WDT_Enable(void);
extern void WDT_Disable(void);
extern void WDT_Reset(void);
extern void WDT_Restart(void);

#endif
