#ifndef DRV_DSU_H
#define DRV_DSU_H

#include "defines.h"

void DSU_Init(void);
void DSU_StartMemoryTest(uint32_t addr, uint32_t len);
bool DSU_IsMemoryOk(void);
bool DSU_IsDone(void);
void DSU_StartCRC(uint32_t addr, uint32_t len);
uint32_t DSU_GetCRC(void);

#endif
