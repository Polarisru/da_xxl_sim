#ifndef DRV_SNUM_H
#define DRV_SNUM_H

#include "defines.h"

#define SNUM_ADDR_1     0x0080A00CU
#define SNUM_ADDR_2     0x0080A040U
#define SNUM_ADDR_3     0x0080A044U
#define SNUM_ADDR_4     0x0080A048U

#define SNUM_LENGTH     16U

void SNUM_Read(uint8_t *data);

#endif
