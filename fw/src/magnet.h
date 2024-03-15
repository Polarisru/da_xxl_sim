#ifndef MAGNET_H
#define MAGNET_H

#include "defines.h"

#define MAGNET_DATA_LENGTH    12U
#define MAGNET_STATUS_LENGTH  6U

#define MAGNET_STS_OCF        (1UL << 5U)
#define MAGNET_STS_COF        (1UL << 4U)
#define MAGNET_STS_LIN        (1UL << 3U)
#define MAGNET_STS_INC        (1UL << 2U)
#define MAGNET_STS_DEC        (1UL << 1U)
#define MAGNET_STS_PAR        (1UL << 0U)

#define MAGNET_ERR_MASK       (MAGNET_STS_OCF | MAGNET_STS_COF | MAGNET_STS_LIN)
#define MAGNET_VALUE_OK       (MAGNET_STS_OCF)

bool MAGNET_Read(uint16_t *val);
bool MAGNET_HasError(void);
uint8_t MAGNET_GetStatus(void);

#endif
