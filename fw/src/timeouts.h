#ifndef TIMEOUTS_H
#define TIMEOUTS_H

#include "defines.h"

/**< Timeouts enumerator */
enum {
  TIMEOUT_TYPE_PARTNER,
  TIMEOUT_TYPE_PARTNER_SHORT,
  TIMEOUT_TYPE_HALL,
  TIMEOUT_TYPE_HOST,
  TIMEOUT_TYPE_ALL
};

void TIMEOUTS_Reset(uint8_t num);
void TIMEOUTS_Inc(uint8_t num);
bool TIMEOUTS_IsActive(uint8_t num);
uint16_t TIMEOUTS_GetValue(uint8_t num);
void TIMEOUTS_Configure(void);

#endif
